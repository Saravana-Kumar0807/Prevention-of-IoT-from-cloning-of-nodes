import math
import time
import network
import espnow
import ubinascii
from machine import Pin
from config import *

class GTI_Node:
    def __init__(self, node_name):
        self.node_name = node_name
        self.region = REGION_MAP.get(node_name, "UNKNOWN")
        self.led = Pin(2, Pin.OUT, value=0)

        self.sta = network.WLAN(network.STA_IF)
        self.sta.active(True)
        self.mac = ubinascii.hexlify(self.sta.config('mac')).decode()
        print(f"{node_name} MAC: {self.mac}")
        self.sta.disconnect()

        self.espnow = espnow.ESPNow()
        self.espnow.active(True)

        self.peers = {}
        for name, mac in PEERS.items():
            if name != self.node_name:
                try:
                    self.espnow.add_peer(mac)
                    self.peers[name] = mac
                    print(f"Added peer {name}: {ubinascii.hexlify(mac).decode()}")
                except Exception as e:
                    print(f"Failed to add peer {name}: {e}")

        self.current_q_index = 0
        self.state = {
            'x': NODE_CONFIG[self.node_name]['p'],
            'status': 'STABLE',
            'neighbors': {},
            'epoch': 0,
            'a_i': 1 / math.sqrt(NODE_CONFIG[self.node_name]['q_list'][0]),
            'convergence_count': 0,
            'last_epoch': time.ticks_ms(),
            'message_counter': 0,
            'previous_neighbors': set()
        }

        self.code_sequence = []
        self.max_history = 20
        self.suspicion_triggered = False

    def next_q(self):
        self.current_q_index = (self.current_q_index + 1) % len(NODE_CONFIG[self.node_name]['q_list'])
        q = NODE_CONFIG[self.node_name]['q_list'][self.current_q_index]
        return q, 1 / math.sqrt(q)

    def reciprocal(self, x):
        return 1/x if x > 1 else x

    def calculate_pseudogeometric_mean(self):
        product = 1.0
        valid_neighbors = 0
        for neighbor, data in self.state['neighbors'].items():
            if REGION_MAP.get(neighbor) == self.region and self.state['epoch'] - data['epoch'] <= 2:
                product *= self.reciprocal(data['code'])
                valid_neighbors += 1
        if valid_neighbors == 0:
            product = self.reciprocal(self.state['x'])
        return product ** self.state['a_i']

    def handle_disturbance(self, reason="topology change"):
        q, a_i = self.next_q()
        self.state.update({
            'a_i': a_i,
            'epoch': self.state['epoch'] + 1,
            'status': 'DISTURBED',
            'convergence_count': 0
        })
        mu_i = self.calculate_pseudogeometric_mean()
        x_new = 1 / mu_i
        if x_new < 1.2:
            x_new = 1.2
        self.state['x'] = x_new
        print(f"! Disturbance amplified to {self.state['x']:.4f} (Reason: {reason}) !")

    def converge_state(self):
        q, a_i = self.next_q()
        self.state.update({
            'a_i': a_i,
            'epoch': self.state['epoch'] + 1
        })
        mu_i = self.calculate_pseudogeometric_mean()
        if self.state['x'] < 1 and abs(mu_i - self.state['x']) < CONVERGENCE_THRESHOLD:
            self.state['convergence_count'] += 1
        else:
            self.state['convergence_count'] = 0
            self.state['x'] = mu_i
        if self.state['convergence_count'] >= STABLE_EPOCHS_REQUIRED:
            self.state['status'] = 'STABLE'
            print("! System converged to stable state !")

    def check_topology_change(self):
        self.process_messages()
        current_neighbors = set(self.state['neighbors'].keys())
        if current_neighbors != self.state['previous_neighbors']:
            added = current_neighbors - self.state['previous_neighbors']
            removed = self.state['previous_neighbors'] - current_neighbors
            if added:
                print(f"[{self.node_name}] New neighbor(s) appeared: {added}")
                self.handle_disturbance("New neighbor(s) appeared")
            if removed:
                print(f"[{self.node_name}] Neighbor(s) disappeared: {removed}")
            self.state['previous_neighbors'] = current_neighbors
            return True
        return False
    
    def process_messages(self):
        while True:
            host, msg = self.espnow.recv(0)
            if not msg:
                break
            try:
                parts = msg.decode().split(':')
                if len(parts) >= 7 and parts[0] == NETWORK_ID:
                    neighbor_name = parts[1]
                    x_val = float(parts[3])
                    epoch = int(parts[6])
                    
                    # Clone detection: check if same name appears with different code in same epoch
                    key = (neighbor_name, epoch)
                    if not hasattr(self, 'clone_check_log'):
                        self.clone_check_log = {}
                    if key in self.clone_check_log:
                        if abs(self.clone_check_log[key] - x_val) > 0.01:  # Different x in same epoch = clone
                            print(f"[{self.node_name}] 🚨 CLONE DETECTED!! {neighbor_name} clone detected in region {REGION_MAP.get(neighbor_name, 'UNKNOWN')} (conflicting x in epoch {epoch})!")
                            self.trigger_alert()
                    else:
                        self.clone_check_log[key] = x_val

                    # Continue storing only latest (overwrites if same name reused)
                    if neighbor_name in self.peers:
                        self.state['neighbors'][neighbor_name] = {
                            'code': x_val,
                            'status': parts[2],
                            'epoch': epoch,
                            'last_seen': time.ticks_ms()
                        }
            except Exception as e:
                print(f"Message error: {e}")

    def broadcast_state(self):
        self.state['message_counter'] += 1
        msg = f"{NETWORK_ID}:{self.node_name}:{self.state['status']}:{self.state['x']:.6f}:" \
              f"{NODE_CONFIG[self.node_name]['p']}:{NODE_CONFIG[self.node_name]['q_list'][self.current_q_index]}:" \
              f"{self.state['epoch']}:{self.state['message_counter']}"
        for mac in self.peers.values():
            try:
                self.espnow.send(mac, msg.encode())
            except Exception as e:
                print(f"Send error: {e}")


    def cleanup_neighbors(self):
        current_time = time.ticks_ms()
        to_remove = [name for name, data in self.state['neighbors'].items()
                     if time.ticks_diff(current_time, data['last_seen']) > 7 * EPOCH_INTERVAL_MS]
        for name in to_remove:
            del self.state['neighbors'][name]
            print(f"[{self.node_name}] Removed stale neighbor after timeout: {name}")

    def update_code_sequence(self):
        if len(self.code_sequence) >= self.max_history:
            self.code_sequence.pop(0)
        self.code_sequence.append(self.state['x'])
        
    def detect_code_anomaly(self):
        if len(self.code_sequence) < 5:
            return

        region_codes = {name: data['code'] for name, data in self.state['neighbors'].items()
                        if REGION_MAP.get(name) == self.region and 'code' in data}

        if len(region_codes) < 2:
            # Avoid false positives when only one neighbor is seen
            return

        region_mean = sum(region_codes.values()) / len(region_codes)
        deviation = abs(self.state['x'] - region_mean)

        if deviation > 0.5:
            print(f"[{self.node_name}] ⚠️ Code anomaly detected in region {self.region}! x={self.state['x']:.4f} vs mean={region_mean:.4f}")

            max_dev = 0
            suspected_node = None
            for name, code in region_codes.items():
                d = abs(code - region_mean)
                if d > max_dev:
                    max_dev = d
                    suspected_node = name

            if suspected_node and not self.suspicion_triggered:
                print(f"[{self.node_name}] 🚨 CLONE DETECTED!! {suspected_node} clone detected in region {self.region}!")
                self.trigger_alert()


    def trigger_alert(self):
        if self.suspicion_triggered:
            return
        self.suspicion_triggered = True
        for _ in range(6):
            self.led.value(1)
            time.sleep_ms(150)
            self.led.value(0)
            time.sleep_ms(150)

    def run(self):
        print(f"\n{self.node_name} initialized in region {self.region}")
        print(f"p={NODE_CONFIG[self.node_name]['p']}, q={NODE_CONFIG[self.node_name]['q_list'][0]}")
        print(f"Peers: {list(self.peers.keys())}\n")

        last_epoch = time.ticks_ms()

        while True:
            current_time = time.ticks_ms()

            if time.ticks_diff(current_time, last_epoch) >= EPOCH_INTERVAL_MS:
                last_epoch = current_time
                self.state['epoch'] += 1

                if self.state['status'] == 'STABLE':
                    if self.check_topology_change():
                        self.handle_disturbance()
                else:
                    self.converge_state()

                self.broadcast_state()
                self.update_code_sequence()
                self.detect_code_anomaly()

                print(f"Epoch {self.state['epoch']}: {self.node_name} x={self.state['x']:.4f} ({self.state['status']}) Region: {self.region} Neighbors: {list(self.state['neighbors'].keys())}")

                self.led.value(1 if self.state['status'] == 'STABLE' else 0)

                if self.state['epoch'] % 10 == 0:
                    self.cleanup_neighbors()

            time.sleep_ms(100)

if __name__ == "__main__":
    node = GTI_Node(NODE_NAME)
    node.run()
