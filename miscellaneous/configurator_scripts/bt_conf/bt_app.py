#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue May  2 13:52:35 2023
@author: Vojtech Vrba (vrbavoj3@fel.cvut.cz)

Implementation of simple Bluetooth application with Wi-Fi management GATT service.

It may be required to:
    1) make sure the device is not blocked, ie.:
        rfkill list all
        rfkill unblock 0
    2) check service status:
        systemctl status bluetooth
        systemctl restart bluetooth
    3) bluetooth is turned on via:
        hciconfig hci0 up
        bluetoothctl -- power on
        bluetoothctl -- discoverable on
"""

import multiprocessing as mp
import signal
import array
import dbus
from gi.repository import GLib
import os
import re

import gatt_server


encode_string = lambda string, encoding="ascii": array.array('B', string.encode(encoding)).tolist()
decode_string = lambda value, encoding="ascii": bytes(value).decode(encoding)
string_to_hexstr = lambda string: ''.join(["%02X" % ord(ch) for ch in string])


class NetplanConfiguration(object):
    NETPLAN_CONFIG_FILE = "/etc/netplan/01-netcfg.yaml"
    AUTOSCRIPTS_DIR = "/home/mrs/git/uav_core/miscellaneous/configurator_scripts/autoscripts"

    def __init__(self):
        self.proc = None
    
    def get_current_id(self):
        if self.proc != None and self.proc.is_alive():
            return 0xFF

        lines = []
        with open(NetplanConfiguration.NETPLAN_CONFIG_FILE, "r") as f:
            lines = f.readlines()
        
        for line in lines:
            matches = re.findall('"mrs_ctu_?(.*)"', line)
            if len(matches) == 1:
                match = matches[0] if len(matches[0]) > 0 else "0"
                try:
                    return int(match)
                except:
                    break
        
        return 0xFF
    
    def set_current_id(self, new_id):
        if self.proc != None and self.proc.is_alive():
            return False
        if new_id == 0:
            filename = "00.sh"
        else:
            filename = "%02X.sh" % (1 << (new_id - 1))
        filepath = os.path.join(NetplanConfiguration.AUTOSCRIPTS_DIR, filename)
        if os.path.exists(filepath):
            cmd = "bash " + filepath
            #proc = subprocess.Popen(cmd.split(' '), stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            #return proc.stdout.read().decode('utf-8').strip()
            self.proc = mp.Process(target=lambda: os.system(cmd))
            self.proc.start()
            return True
        return False



class WifiService(gatt_server.GATTServer_Service):
    UUID = "00" + string_to_hexstr("MRS")

    def __init__(self, bus, index):
        gatt_server.GATTServer_Service.__init__(self, bus, index, self.UUID, True)
        self.add_characteristic(WifiConfigCharacteristic(bus, 0, self))


class WifiConfigCharacteristic(gatt_server.GATTServer_Characteristic):
    UUID = "00000000-0000-0000-0000-" + string_to_hexstr("MRSCTU")

    def __init__(self, bus, index, service):
        gatt_server.GATTServer_Characteristic.__init__(self, bus, index, self.UUID, ['read', 'write'], service)
        self.notifying = False
        self.netplan = NetplanConfiguration()
        self.active_network = 0xFF
        self.update_active()
        #GLib.timeout_add(1000, self.update_active)
        
    def update_active(self):
        prev_network = self.active_network
        self.active_network = self.netplan.get_current_id()
            
        if prev_network != self.active_network and self.active_network != 0xFF:
            self.notify_change()
           
    def notify_change(self):
        if not self.notifying:
            return
        self.PropertiesChanged(gatt_server.BLUEZ_GATT_CHRC_IFACE, {'Value': self.active_network }, []) 
    
    def ReadValue(self, options):
        self.update_active()
        print('[WifiConfigCharacteristic READ] Connected to Wi-Fi with ID:', self.active_network)
        return encode_string(str(self.active_network))

    def WriteValue(self, value, options):
        self.update_active()
        value_str = decode_string(value)
        try:
            new_id = int(value_str)
            if new_id == self.active_network:
                print('[WifiConfigCharacteristic WRITE] Wi-Fi ID is already set to:', new_id)
            else:
                result = self.netplan.set_current_id(new_id)
                if result == True:
                    self.update_active()
                    print('[WifiConfigCharacteristic WRITE] Changed Wi-Fi ID to:', new_id)
                else:
                    raise Exception("Could not change Wi-Fi ID to:" + str(new_id))
        except Exception as e:
            print('[WifiConfigCharacteristic WRITE] Wrong value ', value_str, ':', e)

    def StartNotify(self):
        if self.notifying:
            return
        self.notifying = True
        self.notify_change()

    def StopNotify(self):
        if not self.notifying:
            return
        self.notifying = False


class WifiAdvertisement(gatt_server.GATTServer_Advertisement):
    def __init__(self, bus, index):
        gatt_server.GATTServer_Advertisement.__init__(self, bus, index, 'peripheral')
        self.add_service_uuid(WifiService.UUID)
        self.add_manufacturer_data(0xffff, [ord('M'), ord('R'), ord('S'), 0x0F, 0xEE])
        self.add_service_data('9999', [ord('U'), ord('A'), ord('V'), ord('B'), ord('T')])
        self.include_tx_power = True
       
        
class BluetoothProcessWrapper(mp.Process):
    def __init__(self):
        mp.Process.__init__(self)
    
    def run(self):
        singal_handler = lambda _signo, _stack_frame: self.stop()
        signal.signal(signal.SIGTERM, singal_handler)
        signal.signal(signal.SIGINT, singal_handler)
        
        self.server = gatt_server.GATTServer([WifiService], [WifiAdvertisement])
        print("Starting Bluetooth app...")
        self.server.start()
        
    def stop(self):
        if "server" in dir(self):
            print("Terminating Bluetooth app...")
            self.server.stop()
            self.server = None
            try:
                self.kill()
            except:
                pass


def signal_handler(sig, frame):
    bt_proc.stop()


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    
    bt_proc = BluetoothProcessWrapper()
    bt_proc.start()
    bt_proc.join()

