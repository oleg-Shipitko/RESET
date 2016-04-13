import typeConvertor
import packetBuilder
import crc
import binascii

class ParsePacket(object):

	synchronization = 'FA'	
	adress = 'FA'
	typeConvertor = typeConvertor.TypeConvertor()
	crcCalculator = crc.CrcCalculator()
	listOfComands =  packetBuilder.CommandsList()

	def __init__(self, byteString):
		self.string = byteString
		self.byteArray = self.GetByteArrayFromRecievedString()
		self.recievedSynchronizationByte = self.byteArray[0]
		self.recievedAdress = self.byteArray[1]
		self.packetLenght = self.byteArray[2]
		self.command = self.byteArray[3]
		self.reply = self.GetReply()
		self.recievedcrc = self.typeConvertor.IntToHex(self.byteArray[len(self.byteArray) - 2]) + self.typeConvertor.IntToHex(self.byteArray[len(self.byteArray) - 1])

		self.CheckCrc()

	def GetReply(self):
		if self.listOfComands.getCurentCoordinates == self.command:
			recievedPametersString = binascii.hexlify(self.byteArray[4:len(self.byteArray) - 2])
			invertedParametersString = self.typeConvertor.InvertStringArray(recievedPametersString)
			coordinatesList = []
 			coordinatesList.append(self.typeConvertor.HexToFloat(invertedParametersString[16:24]))
 			coordinatesList.append(self.typeConvertor.HexToFloat(invertedParametersString[8:16]))
 			coordinatesList.append(self.typeConvertor.HexToFloat(invertedParametersString[0:8]))
			return coordinatesList
			
		if self.listOfComands.closeCubeCollector == self.command:
			recievedPametersString = binascii.hexlify(self.byteArray[4:len(self.byteArray) - 2])			
			return recievedPametersString

		return self.byteArray[4:len(self.byteArray) - 2][:-1]

	def CheckCrc(self):
		calculatedCrc = self.crcCalculator.GetTwoBytesFromCrc32(self.byteArray[0:len(self.byteArray) - 2])
		if calculatedCrc != self.recievedcrc:
			raise Exception('Expected crc: ' + self.recievedcrc + ', but calculated: ' + calculatedCrc)

	def GetByteArrayFromRecievedString(self):
		byteString = ""
		for index in range(0, len(self.string)):
			hexSymbol = self.typeConvertor.IntToHex(ord(str(self.string[index])))
			#print str(index) + ': ' + hexSymbol			
 			byteString = byteString + hexSymbol
		byteString = bytearray.fromhex(byteString)
		#print 'end'
		return byteString

 
