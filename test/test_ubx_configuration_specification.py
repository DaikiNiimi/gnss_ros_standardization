#!/usr/bin/env python3
"""Selected outgoing configuration bytes and ACK semantics from U2 sections
32.8, 32.10.13/.18/.19/.27/.29, 32.16.13 and U1/U3/U5 CFG-VALSET/key tables.
The fake transport is shared, but no production constants/encoder are used to
derive expected bytes. G9/G10 are repository modes, not firmware certification.
"""
import struct
import unittest
import test_driver_configuration as transport


class ConfigurationSpecification(unittest.TestCase):
    def packets(self, generation):
        receiver=transport.DriverConfiguration()
        # U2 pp208-209: GPS and GLO L1 enabled, eight reserved /16 max channels
        # each within the32-channel receiver. Disabling GLO must preserve L1 mask.
        gnss=bytes([0,32,32,2])+struct.pack('<4BI',0,8,16,0,0x10001)+struct.pack('<4BI',6,8,16,0,0x10001)
        params=['-p','generation:='+generation,'-p','rate_hz:=10',
                '-p','dynamic_model:=automotive','-p','navsys.glo:=false']
        for constellation in ('gps','gal','bds','qzs','sbs','irn'):
            params+=['-p','navsys.'+constellation+':=true']
        for name in ('rawx','sfrbx','nav_pvt','nav_cov','nav_clock','nav_dop',
                     'nav_posecef','nav_velecef','nav_hpposecef','nav_hpposllh','esf_raw','esf_ins'):
            params+=['-p','messages.'+name+':=true']
        for name in ('gga','rmc','gsa','gst'):
            params+=['-p','messages.nmea_'+name+':=false']
        receiver.receiver('ubx','ack',params,gnss_poll_payload=gnss)
        result={}
        for packet in receiver.last_packets:
            n=struct.unpack_from('<H',packet,4)[0]
            self.assertEqual(len(packet),n+8)
            # Independent weighted-sum form of the UBX Fletcher checksum.
            body=packet[2:-2]
            self.assertEqual(packet[-2:],bytes([sum(body)%256,
                sum((len(body)-i)*b for i,b in enumerate(body))%256]))
            result.setdefault(tuple(packet[2:4]),[]).append(packet[6:-2])
        self.assertEqual(result[(10,4)],[b''])  # MON-VER poll, U2 p355
        return result

    def test_legacy_message_rate_navigation_and_gnss_layout(self):
        packets=self.packets('G9')
        self.assertEqual(packets[(6,8)],[struct.pack('<HHH',100,1,0)])
        # Only dynModel is selected by mask bit0; other NAV5 bytes aren't applied.
        self.assertEqual(len(packets[(6,36)][0]),36)
        self.assertEqual(packets[(6,36)][0][:3],b'\x01\x00\x04')
        for payload in packets[(6,1)]: self.assertEqual(len(payload),8)
        for message in (b'\x02\x15',b'\x02\x13'):
            self.assertIn(message+b'\x01\x01\x01\x01\x00\x00',packets[(6,1)])
        self.assertEqual(packets[(6,62)][0],b'')
        configured=packets[(6,62)][1]
        self.assertEqual(len(configured),4+8*configured[3])
        self.assertEqual(configured,bytes([0,32,32,2])+
                         struct.pack('<4BI',0,8,16,0,0x10001)+struct.pack('<4BI',6,8,16,0,0x10000))

    def test_valset_types_values_ram_layer_and_unacknowledged_reset(self):
        packets=self.packets('G10')
        self.assertEqual(packets[(6,4)],[b'\x00\x00\x02\x00'])
        values={}
        for payload in packets[(6,138)]:
            self.assertEqual(payload[:4],b'\x00\x01\x00\x00')  # version0, RAM
            pos=4
            while pos<len(payload):
                self.assertGreaterEqual(len(payload)-pos,5)
                key=struct.unpack_from('<I',payload,pos)[0];pos+=4
                width={1:1,2:1,3:2,4:4,5:8}[key>>28]
                self.assertLessEqual(pos+width,len(payload))
                value=int.from_bytes(payload[pos:pos+width],'little');pos+=width
                if key in values:self.assertEqual(values[key],value)
                values[key]=value
        # U1/U3/U5 key tables: every currently requested group and message.
        # U5 confirms X20 L5/B1C/B2a/NavIC keys absent from F9 HPG U1.
        expected={0x30210001:100,0x30210002:1,0x20110021:4,
                  0x1031001f:1,0x10310001:1,0x10310003:1,0x10310004:1,
                  0x10310020:1,0x10310005:1,
                  0x10310021:1,0x10310007:1,0x10310009:1,0x1031000a:1,
                  0x10310022:1,0x1031000d:1,0x1031000e:1,0x1031000f:1,0x10310028:1,
                  0x10310024:1,0x10310012:1,0x10310014:1,0x10310015:1,0x10310017:1,
                  0x10310025:0,0x10310018:0,0x1031001a:0,
                  0x10310026:1,0x1031001d:1}
        # Base I2C keys plus1/2/3 give UART1/2/USB. Check all three requested
        # ports rather than accepting a correct value written to a wrong port.
        for base in (0x209102a4,0x20910231,0x20910006,0x20910038,0x20910083,
                     0x20910065,0x20910024,0x2091003d,0x2091002e,0x20910033,
                     0x20910114,0x2091029f):
            for port in (1,2,3):expected[base+port]=1
        for base in (0x209100ba,0x209100ab,0x209100bf,0x209100d3,
                     0x209100c9,0x209100b0,0x209100d8,0x209100c4):
            for port in (1,2,3):expected[base+port]=0
        for key,value in expected.items():
            with self.subTest(key=hex(key)):self.assertEqual(values.get(key),value)

    def test_product_unsupported_key_is_reported_as_partial_configuration(self):
        receiver=transport.DriverConfiguration()
        # V6 only treats a rejected enabled output as incomplete. The fixture
        # NAKs ESF-INS (0x20910114 + port), so request that optional output.
        text=receiver.receiver('ubx','valset_nak',
                               ['-p','generation:=G10','-p','messages.esf_ins:=true'])
        self.assertIn('Receiver configuration is incomplete',text)
        self.assertNotIn('Receiver configuration complete',text)
        # Subsequent supported commands still receive ACK and initialization succeeds.
        for port in (1,2,3):
            # After the aggregate NAK, RAWX has its own acknowledged transaction.
            expected=struct.pack('<BBBBIB',0,1,0,0,0x209102a4+port,1)
            self.assertTrue(any(p[2:4]==bytes([6,138]) and p[6:-2]==expected for p in receiver.last_packets))

    def test_documented_ack_nak_matching_and_truncated_poll_rejection(self):
        receiver=transport.DriverConfiguration()
        for behavior in ('wrong_ack','truncated'):
            with self.subTest(behavior=behavior):receiver.receiver('ubx',behavior)
        # NAK names the exact CFG-NAV5 command. Optional failure must not be
        # reported as a complete receiver configuration.
        text=receiver.receiver('ubx','optional_nak',['-p','generation:=G9'])
        self.assertIn('Receiver configuration is incomplete',text)


if __name__=='__main__':
    result=unittest.TextTestRunner(verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromTestCase(ConfigurationSpecification))
    raise SystemExit(0 if result.wasSuccessful() else 1)
