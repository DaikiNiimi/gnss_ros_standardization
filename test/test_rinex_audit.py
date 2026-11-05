#!/usr/bin/env python3
import io
import unittest
from audit_rinex_observations import read_header,epochs,compare

class RinexAudit(unittest.TestCase):
    def test_strict_presence_distinguishes_blank_from_explicit_zero(self):
        stream=io.StringIO(self.header()+'> 2024 02 04 00 00  0.0000000  0  1\n'+
            'C59'+f'{21000000.:14.3f}  '+' '*16+f'{0.:14.3f}  '+' '*16+'\n')
        header=read_header(stream);source=list(epochs(stream,header));ns,signals=source[0]
        self.assertEqual(signals,{('C59','2I'):{'C':21000000.,'D':0.}})
        values={'C':21000000.,'L':float('nan'),'D':0.,'S':float('nan'),'lli':0,'freq':255}
        def audit():return compare(iter(source),iter([(ns,{('C59','2I'):values},0,0)]),
                                  header,'unchecked',require_nan_missing=True)
        self.assertTrue(audit()['passed'])
        values['L']=0.
        self.assertEqual(audit()['counts']['blank_not_nan_L'],1)
        values['L']=float('nan');values['D']=float('nan')
        self.assertEqual(audit()['counts']['explicit_zero_D'],1)

    def header(self):
        def row(text,label):return text.ljust(60)+label+'\n'
        return (row('     3.04           OBSERVATION DATA    M','RINEX VERSION / TYPE')+
                row('C    4 C2I L2I D2I S2I','SYS / # / OBS TYPES')+
                row('  2024     2     4     0     0    0.0000000     GPS','TIME OF FIRST OBS')+
                row('','END OF HEADER'))
    def test_original_high_prn_and_every_signal_are_required(self):
        stream=io.StringIO(self.header()+'> 2024 02 04 00 00  0.0000000  0  1\n'+
                          'C63'+f'{21000000.125:14.3f}  '+f'{110000000.25:14.3f}3 '+
                          f'{-1234.5:14.3f}  '+f'{45.:14.3f}  '+'\n')
        header=read_header(stream);data=list(epochs(stream,header));ns,signals=data[0]
        self.assertEqual(signals,{('C63','2I'):{'C':21000000.125,'L':110000000.25,'lli':3,'D':-1234.5,'S':45.}})
        good={('C63','2I'):{**signals[('C63','2I')],'freq':255}}
        stamp=315964800_000_000_000+ns
        self.assertTrue(compare(iter(data),iter([(ns,good,stamp,stamp)]),header,'gpst')['passed'])
        dropped=compare(iter(data),iter([(ns,{},stamp,stamp)]),header,'gpst')
        self.assertFalse(dropped['passed']);self.assertEqual(dropped['counts']['missing_signal'],1)
        self.assertFalse(compare(iter(data),iter([(ns+604800_000_000_000,good,stamp,stamp)]),header,'gpst')['passed'])
    def test_no_silent_event_or_time_system_assumptions(self):
        with self.assertRaises(ValueError):read_header(io.StringIO(self.header().replace('GPS','UTC')))
        stream=io.StringIO(self.header()+'> 2024 02 04 00 00  0.0000000  4  0\n')
        header=read_header(stream)
        with self.assertRaises(ValueError):list(epochs(stream,header))
    def test_lli_and_missing_field_invention_fail_acceptance(self):
        expected={('C59','2I'):{'C':1.,'lli':3}}
        actual={('C59','2I'):{'C':1.,'L':0.,'D':float('nan'),'S':0.,'lli':3,'freq':255}}
        header={'glo_channels':{}}
        self.assertTrue(compare(iter([(0,expected)]),iter([(0,actual,0,0)]),header,'unchecked')['passed'])
        actual[('C59','2I')]['lli']=1;actual[('C59','2I')]['D']=12.
        result=compare(iter([(0,expected)]),iter([(0,actual,0,0)]),header,'unchecked')
        self.assertEqual(result['counts']['lli'],1);self.assertEqual(result['counts']['invented_D'],1)
if __name__=='__main__':unittest.main()
