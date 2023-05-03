#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue May  2 13:23:48 2023
@author: Vojtech Vrba (vrbavoj3@fel.cvut.cz)

Generic tools for interfacing Bluez Linux driver.
Adapted from: https://github.com/Jumperr-labs/python-gatt-server
"""

import dbus, dbus.exceptions, dbus.mainloop.glib, dbus.service
from gi.repository import GLib
import functools
import socket


BLUEZ_SERVICE_NAME                  = 'org.bluez'
BLUEZ_LE_ADVERTISING_MANAGER_IFACE  = 'org.bluez.LEAdvertisingManager1'
BLUEZ_LE_ADVERTISEMENT_IFACE        = 'org.bluez.LEAdvertisement1'
BLUEZ_GATT_MANAGER_IFACE            = 'org.bluez.GattManager1'
BLUEZ_GATT_SERVICE_IFACE            = 'org.bluez.GattService1'
BLUEZ_GATT_CHRC_IFACE               = 'org.bluez.GattCharacteristic1'
BLUEZ_GATT_DESC_IFACE               = 'org.bluez.GattDescriptor1'
BLUEZ_EXCEPTION_NOT_SUPPORTED       = 'org.bluez.Error.NotSupported'
BLUEZ_EXCEPTION_NOT_PERMITTED       = 'org.bluez.Error.NotPermitted'
BLUEZ_EXCEPTION_INVALID_VALUE_LEN   = 'org.bluez.Error.InvalidValueLength'
BLUEZ_EXCEPTION_FAILED              = 'org.bluez.Error.Failed'

DBUS_OM_IFACE                       = 'org.freedesktop.DBus.ObjectManager'
DBUS_PROP_IFACE                     = 'org.freedesktop.DBus.Properties'
DBUS_EXCEPTION_INVALID_ARGS         = 'org.freedesktop.DBus.Error.InvalidArgs'


class GATTServer_InvalidArgsException(dbus.DBusException):
    _dbus_error_name = DBUS_EXCEPTION_INVALID_ARGS


class GATTServer_NotSupportedException(dbus.DBusException):
    _dbus_error_name = BLUEZ_EXCEPTION_NOT_SUPPORTED


class GATTServer_NotPermittedException(dbus.DBusException):
    _dbus_error_name = BLUEZ_EXCEPTION_NOT_PERMITTED


class GATTServer_InvalidValueLengthException(dbus.DBusException):
    _dbus_error_name = BLUEZ_EXCEPTION_INVALID_VALUE_LEN


class GATTServer_FailedException(dbus.DBusException):
    _dbus_error_name = BLUEZ_EXCEPTION_FAILED
    

class GATTServer_Application(dbus.service.Object):
    """
    org.bluez.GattApplication1 interface implementation
    """
    def __init__(self, bus, services):
        self.path = '/'
        self.services = []
        dbus.service.Object.__init__(self, bus, self.path)
        for i, service in enumerate(services):
            self.add_service(service(bus, i))

    def get_path(self):
        return dbus.ObjectPath(self.path)

    def add_service(self, service):
        self.services.append(service)

    @dbus.service.method(DBUS_OM_IFACE, out_signature='a{oa{sa{sv}}}')
    def GetManagedObjects(self):
        response = {}

        for service in self.services:
            response[service.get_path()] = service.get_properties()
            chrcs = service.get_characteristics()
            for chrc in chrcs:
                response[chrc.get_path()] = chrc.get_properties()
                descs = chrc.get_descriptors()
                for desc in descs:
                    response[desc.get_path()] = desc.get_properties()

        return response
    
    def destroy(self):
        for serv in self.services:
            serv.destroy()
        self.remove_from_connection()


class GATTServer_Service(dbus.service.Object):
    """
    org.bluez.GattService1 interface implementation
    """
    PATH_BASE = '/org/bluez/example/service'

    def __init__(self, bus, index, uuid, primary):
        self.path = self.PATH_BASE + str(index)
        self.bus = bus
        self.uuid = uuid
        self.primary = primary
        self.characteristics = []
        dbus.service.Object.__init__(self, bus, self.path)

    def get_properties(self):
        return {
                BLUEZ_GATT_SERVICE_IFACE: {
                        'UUID': self.uuid,
                        'Primary': self.primary,
                        'Characteristics': dbus.Array(
                                self.get_characteristic_paths(),
                                signature='o')
                }
        }

    def get_path(self):
        return dbus.ObjectPath(self.path)

    def add_characteristic(self, characteristic):
        self.characteristics.append(characteristic)

    def get_characteristic_paths(self):
        result = []
        for chrc in self.characteristics:
            result.append(chrc.get_path())
        return result

    def get_characteristics(self):
        return self.characteristics

    @dbus.service.method(DBUS_PROP_IFACE, in_signature='s', out_signature='a{sv}')
    def GetAll(self, interface):
        if interface != BLUEZ_GATT_SERVICE_IFACE:
            raise GATTServer_InvalidArgsException()

        return self.get_properties()[BLUEZ_GATT_SERVICE_IFACE]
    
    def destroy(self):
        for chrc in self.characteristics:
            chrc.destroy()
        self.remove_from_connection()


class GATTServer_Characteristic(dbus.service.Object):
    """
    org.bluez.GattCharacteristic1 interface implementation
    """
    def __init__(self, bus, index, uuid, flags, service):
        self.path = service.path + '/char' + str(index)
        self.bus = bus
        self.uuid = uuid
        self.service = service
        self.flags = flags
        self.descriptors = []
        dbus.service.Object.__init__(self, bus, self.path)

    def get_properties(self):
        return {
                BLUEZ_GATT_CHRC_IFACE: {
                        'Service': self.service.get_path(),
                        'UUID': self.uuid,
                        'Flags': self.flags,
                        'Descriptors': dbus.Array(
                                self.get_descriptor_paths(),
                                signature='o')
                }
        }

    def get_path(self):
        return dbus.ObjectPath(self.path)

    def add_descriptor(self, descriptor):
        self.descriptors.append(descriptor)

    def get_descriptor_paths(self):
        result = []
        for desc in self.descriptors:
            result.append(desc.get_path())
        return result

    def get_descriptors(self):
        return self.descriptors

    @dbus.service.method(DBUS_PROP_IFACE,
                         in_signature='s',
                         out_signature='a{sv}')
    def GetAll(self, interface):
        if interface != BLUEZ_GATT_CHRC_IFACE:
            raise GATTServer_InvalidArgsException()

        return self.get_properties()[BLUEZ_GATT_CHRC_IFACE]

    @dbus.service.method(BLUEZ_GATT_CHRC_IFACE, in_signature='a{sv}', out_signature='ay')
    def ReadValue(self, options):
        #print('Default ReadValue called, returning error')
        raise GATTServer_NotSupportedException()

    @dbus.service.method(BLUEZ_GATT_CHRC_IFACE, in_signature='aya{sv}')
    def WriteValue(self, value, options):
        #print('Default WriteValue called, returning error')
        raise GATTServer_NotSupportedException()

    @dbus.service.method(BLUEZ_GATT_CHRC_IFACE)
    def StartNotify(self):
        #print('Default StartNotify called, returning error')
        raise GATTServer_NotSupportedException()

    @dbus.service.method(BLUEZ_GATT_CHRC_IFACE)
    def StopNotify(self):
        #print('Default StopNotify called, returning error')
        raise GATTServer_NotSupportedException()

    @dbus.service.signal(DBUS_PROP_IFACE, signature='sa{sv}as')
    def PropertiesChanged(self, interface, changed, invalidated):
        pass
    
    def destroy(self):
        for desc in self.descriptors:
            desc.remove_from_connection()
        self.remove_from_connection()


class GATTServer_Descriptor(dbus.service.Object):
    """
    org.bluez.GattDescriptor1 interface implementation
    """
    def __init__(self, bus, index, uuid, flags, characteristic):
        self.path = characteristic.path + '/desc' + str(index)
        self.bus = bus
        self.uuid = uuid
        self.flags = flags
        self.chrc = characteristic
        dbus.service.Object.__init__(self, bus, self.path)

    def get_properties(self):
        return {
                BLUEZ_GATT_DESC_IFACE: {
                        'Characteristic': self.chrc.get_path(),
                        'UUID': self.uuid,
                        'Flags': self.flags,
                }
        }

    def get_path(self):
        return dbus.ObjectPath(self.path)

    @dbus.service.method(DBUS_PROP_IFACE,  in_signature='s', out_signature='a{sv}')
    def GetAll(self, interface):
        if interface != BLUEZ_GATT_DESC_IFACE:
            raise GATTServer_InvalidArgsException()

        return self.get_properties()[BLUEZ_GATT_DESC_IFACE]

    @dbus.service.method(BLUEZ_GATT_DESC_IFACE, in_signature='a{sv}', out_signature='ay')
    def ReadValue(self, options):
        #print('Default ReadValue called, returning error')
        raise GATTServer_NotSupportedException()

    @dbus.service.method(BLUEZ_GATT_DESC_IFACE, in_signature='aya{sv}')
    def WriteValue(self, value, options):
        #print('Default WriteValue called, returning error')
        raise GATTServer_NotSupportedException()


class GATTServer_Advertisement(dbus.service.Object):
    PATH_BASE = '/org/bluez/example/advertisement'

    def __init__(self, bus, index, advertising_type):
        self.path = self.PATH_BASE + str(index)
        self.bus = bus
        self.ad_type = advertising_type
        self.service_uuids = None
        self.manufacturer_data = None
        self.solicit_uuids = None
        self.service_data = None
        self.include_tx_power = None
        dbus.service.Object.__init__(self, bus, self.path)

    def get_properties(self):
        properties = dict()
        properties['Type'] = self.ad_type
        if self.service_uuids is not None:
            properties['ServiceUUIDs'] = dbus.Array(self.service_uuids, signature='s')
        if self.solicit_uuids is not None:
            properties['SolicitUUIDs'] = dbus.Array(self.solicit_uuids, signature='s')
        if self.manufacturer_data is not None:
            properties['ManufacturerData'] = dbus.Dictionary(self.manufacturer_data, signature='qv')
        if self.service_data is not None:
            properties['ServiceData'] = dbus.Dictionary(self.service_data, signature='sv')
        if self.include_tx_power is not None:
            properties['IncludeTxPower'] = dbus.Boolean(self.include_tx_power)
        properties["LocalName"] = dbus.String(socket.gethostname())
        return {BLUEZ_LE_ADVERTISEMENT_IFACE: properties}

    def get_path(self):
        return dbus.ObjectPath(self.path)

    def add_service_uuid(self, uuid):
        if not self.service_uuids:
            self.service_uuids = []
        self.service_uuids.append(uuid)

    def add_solicit_uuid(self, uuid):
        if not self.solicit_uuids:
            self.solicit_uuids = []
        self.solicit_uuids.append(uuid)

    def add_manufacturer_data(self, manuf_code, data):
        if not self.manufacturer_data:
            self.manufacturer_data = dbus.Dictionary({}, signature='qv')
        self.manufacturer_data[manuf_code] = dbus.Array(data, signature='y')

    def add_service_data(self, uuid, data):
        if not self.service_data:
            self.service_data = dbus.Dictionary({}, signature='sv')
        self.service_data[uuid] = dbus.Array(data, signature='y')

    @dbus.service.method(DBUS_PROP_IFACE, in_signature='s', out_signature='a{sv}')
    def GetAll(self, interface):
        if interface != BLUEZ_LE_ADVERTISEMENT_IFACE:
            raise GATTServer_InvalidArgsException()
        return self.get_properties()[BLUEZ_LE_ADVERTISEMENT_IFACE]

    @dbus.service.method(BLUEZ_LE_ADVERTISEMENT_IFACE, in_signature='', out_signature='')
    def Release(self):
        print('%s: Released!' % self.path)
        
    def destroy(self):
        self.remove_from_connection()
        

class GATTServer(object):
    def __init__(self, services, advertisements, adapter_name=''):
        dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
        self.bus = dbus.SystemBus()
        self.mainloop = GLib.MainLoop()
        self.adapter_name = adapter_name
        
        self.adapter = self.find_adapter()
        if not self.adapter:
            raise Exception('GattManager1 interface not found')
            
        self.service_manager = self.get_service_manager()
        self.ad_manager = self.get_advertisement_manager()
        
        self.app = GATTServer_Application(self.bus, services)
        self.service_manager.RegisterApplication(self.app.get_path(), {}, reply_handler=lambda: self.register_app_cb(), error_handler=functools.partial(lambda ml, err: self.register_app_error_cb(ml, err), self.mainloop))
        
        self.advertisements = list()
        for i, ad in enumerate(advertisements):
            ad_inst = ad(self.bus, i)
            self.ad_manager.RegisterAdvertisement(ad_inst.get_path(), {}, reply_handler=lambda: self.register_ad_cb(), error_handler=functools.partial(lambda ml, err: self.register_ad_error_cb(ml, err), self.mainloop))
            self.advertisements.append(ad_inst)
        
    def find_adapter(self):
        remote_om = dbus.Interface(self.bus.get_object(BLUEZ_SERVICE_NAME, '/'), DBUS_OM_IFACE)
        objects = remote_om.GetManagedObjects()
        for o, props in objects.items():
            if BLUEZ_GATT_MANAGER_IFACE in props.keys():
                if '/' + self.adapter_name in o:
                    return o
        return None
    
    def get_service_manager(self):
        return dbus.Interface(self.bus.get_object(BLUEZ_SERVICE_NAME, self.adapter), BLUEZ_GATT_MANAGER_IFACE)
    
    def get_advertisement_manager(self):
        return dbus.Interface(self.bus.get_object(BLUEZ_SERVICE_NAME, self.adapter), BLUEZ_LE_ADVERTISING_MANAGER_IFACE)

    def register_app_cb(self):
        print('GATT application registered')
    
    def register_app_error_cb(self, mainloop, error):
        print('Failed to register application: ' + str(error))
        mainloop.quit()
        
    def register_ad_cb(self):
        print('Advertisement registered')
    
    def register_ad_error_cb(self, mainloop, error):
        print('Failed to register advertisement: ' + str(error))
        mainloop.quit()
    
    def start(self):
        self.mainloop.run()
    
    def stop(self):
        self.mainloop.quit()
        self.app.destroy()
        for ad in self.advertisements:
            ad.destroy()



