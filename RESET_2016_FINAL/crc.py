import numpy
import sys

class CrcCalculator(object):

	def __init__(self):
		self.crcPoly = numpy.uint64(0x4C11DB7)
		self.crcTable = [numpy.uint32(0)]*512		

	def GetTwoBytesFromCrc32(self, data):		 
		fullCRC = str(hex(self.GetCrc32(data)))[:-1]
		lastBytes = str(fullCRC)[len(fullCRC) - 2:len(fullCRC) + 1]
		almostLastBytes = str(fullCRC)[len(fullCRC) - 4:len(fullCRC) - 2]
		return lastBytes + almostLastBytes

	def GetCrc32(self, data):
		crc = numpy.uint32(0xFFFFFFFF)
		for i in range(len(data)):
			crc = self.RecountCrc(data[i], crc)
		return crc

	def RecountCrc(self, byte, crcOld):
		crc = numpy.uint32(crcOld) ^ numpy.uint32(byte)
		for i in range(32):
			if int((crc & numpy.uint32(0x80000000))) is 0:
				crc = (crc << numpy.uint32(1))&numpy.uint32(0xFFFFFFFF)
			else:
				crc = ((crc << numpy.uint32(1))&numpy.uint32(0xFFFFFFFF)) ^ numpy.uint32(self.crcPoly)
		return crc

			
		
