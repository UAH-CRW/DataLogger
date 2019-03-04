#!/usr/bin/env python3
#Convert binary log files from the vehicle data logger into standard CSV files
#Daniel Corey
import struct, sys

packet_len = 32
def convertdata(rawdata, outfile):
	first_packet = rawdata.find(b'>')
	#Check that there's another '>' 1 packet after this one (minimize chances of false packet detect)
	if chr(rawdata[first_packet + packet_len]) != '>':
		print('Warning: skipping first packet, found {0}'.format(chr(rawdata[first_packet + packet_len])))
		offset = first_packet + 1
		return convertdata(rawdata[offset:], outfile)
	
	rawdata = rawdata[first_packet:]
	fout = open(outfile, 'w')
	for i in range(0, len(rawdata), packet_len):
		start = i
		stop = start + packet_len
		try:
			data = struct.unpack('cIhhhhhhhhhi', rawdata[start:stop])
		except:
			print('{0} - {1}; {2}'.format(start, stop, len(rawdata)))
			break
		#print(data)
		fout.write(','.join([str(i) for i in data[1:]]) + '\n')
	fout.close()

if __name__ == '__main__':
	f = sys.argv[1]
	with open(f + '.BIN', 'rb') as infile:
		convertdata(infile.read(), f + '.CSV')