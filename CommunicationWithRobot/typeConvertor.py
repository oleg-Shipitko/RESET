import struct
# This class was introduced because Python can't understand itself. Method hex() returns 0x_something_ and in the same time 
# method fromhex() expected _something_ without 0x (strictly 2 numbers). Example: hex(1) return 0x1 but 
# we have to pass 01 to method fromhex()

class TypeConvertor(object):

	@staticmethod
	def IntToHex(intNumber):
		hexNumber = hex(intNumber)[2:]
		if len(hexNumber) is 1:
			hexNumber = '0' + hexNumber
		if int(hexNumber, 16) is not intNumber:
			raise Exception('Number ' + str(intNumber) + ' was converted to hex unsuccessfully.')			
		return hexNumber

	@staticmethod
	def FloatToHex(floatNumber):
		if floatNumber == 0:
			return '00000000'
		hexValue = hex(struct.unpack('<I', struct.pack('<f', floatNumber))[0])
		return TypeConvertor.InvertStringArray(hexValue[2:len(hexValue)])

	@staticmethod
	def HexToFloat(hexNumber):		
		return struct.unpack('!f', hexNumber.decode('hex'))[0]

	# Representation of float numbers in C++ different from representation in python. 
	# That's why we have to invert string array	
	@staticmethod 
	def InvertStringArray(stringArray):
		invertedArray = ''
		for index in range(0, len(stringArray)):
			if index % 2 == 1:
				invertedArray = stringArray[index - 1: index + 1] + invertedArray
		return invertedArray		




