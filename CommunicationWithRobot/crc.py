class CrcCalculator(object):

	def __init__(self):
		self.crcPoly = 0x4C11DB7
		self.crcTable = [0]*512

	def GetTwoBytesFromCrc32(self, data):
		fullCRC = str(hex(self.GetCrc32(data)))
		lastBytes = str(fullCRC)[len(fullCRC) - 2:len(fullCRC) + 1]
		almostLastBytes = str(fullCRC)[len(fullCRC) - 4:len(fullCRC) - 2]
		return lastBytes + almostLastBytes

	def GetCrc32(self, data):
		crc = 0xFFFFFFFF
		for i in range(0, len(data)):
			crc = self.RecountCrc(data[i], crc)			
		return crc

	def RecountCrc(self, byte, crcOld):
		crc = crcOld ^ byte
		for i in range(0, 32):
			if (crc & 0x80000000) is 0:
				crc = (crc << 1)&0xFFFFFFFF
			else:
				crc = ((crc << 1)&0xFFFFFFFF) ^ self.crcPoly
		return crc

			
		
