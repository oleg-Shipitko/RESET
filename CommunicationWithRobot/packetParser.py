import typeConvertor
import crc

class ParsePacket(object):

	synchronization = 'FA'	
	adress = 'FA'
	typeConvertor = typeConvertor.TypeConvertor()
	crcCalculator = crc.CrcCalculator()

	def __init__(self, byteString):
		self.string = byteString
		self.byteArray = self.GetByteArrayFromRecievedString()
		self.recievedSynchronizationByte = self.byteArray[0]
		self.recievedAdress = self.byteArray[1]
		self.packetLenght = self.byteArray[2]
		self.command = self.byteArray[3]
		self.reply = self.byteArray[4:len(self.byteArray) - 2]
		self.recievedcrc = self.typeConvertor.IntToHex(self.byteArray[len(self.byteArray) - 2]) + self.typeConvertor.IntToHex(self.byteArray[len(self.byteArray) - 1])

		self.CheckCrc()

	def CheckCrc(self):
		calculatedCrc = self.crcCalculator.GetTwoBytesFromCrc32(self.byteArray[0:len(self.byteArray) - 2])
		if calculatedCrc != self.recievedcrc:
			raise Exception('Expected crc: ' + self.recievedcrc + ', but calculated: ' + calculatedCrc)

	def GetByteArrayFromRecievedString(self):
		byteString = ""
		for index in range(0, len(self.string)):
			hexSymbol = self.typeConvertor.IntToHex(ord(str(self.string[index])))			
 			byteString = byteString + hexSymbol
		byteString = bytearray.fromhex(byteString)
		return byteString

 