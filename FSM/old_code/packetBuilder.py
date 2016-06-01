import typeConvertor
import crc

class BuildPacket(object):

	synchronization = 'FA'	
	adress = 'AF'
	typeConvertor = typeConvertor.TypeConvertor()
	crcCalculator = crc.CrcCalculator()
	
	def __init__(self, comand, parameters = None):
		#print comand
		#rint parameters
		self.comand = self.typeConvertor.IntToHex(comand)		
		self.parameters = self.GetParametersInHexForm(comand, parameters)
		self.packetLenght = self.typeConvertor.IntToHex(6 + len(self.parameters) / 2)
		self.crc = self.CalculateCrc()
		self.bytearray = self.GetBytePacket()		
		
	def CalculateCrc(self):
		concatinatedString = self.synchronization +  self.adress + self.packetLenght +  self.comand +  self.parameters
		byteString = bytearray.fromhex(concatinatedString)
		return self.crcCalculator.GetTwoBytesFromCrc32(byteString)
		

	def GetBytePacket(self):
		concatinatedString = self.synchronization +  self.adress + self.packetLenght +  self.comand +  self.parameters + self.crc
		return bytearray.fromhex(concatinatedString)

	def GetParametersInHexForm(self, comand, parameters):
		if comand == CommandsList.echo:
			return self.GetByteStringFormString(parameters)
		elif comand == CommandsList.setCoordinates or comand == CommandsList.setCorectCoordinates:
			return self.GetByteStringFromFloatList(parameters)
		elif comand == CommandsList.dutyCycle:
			return self.typeConvertor.IntToHex(parameters[0]) + self.typeConvertor.FloatToHex(parameters[1])
		elif comand == CommandsList.setManipulatorAngle:
			return self.typeConvertor.FloatToHex(parameters)
		elif comand == CommandsList.setDirectionBit or comand == CommandsList.removeDirectionBit:
			return self.typeConvertor.IntToHex(parameters)
		elif comand == CommandsList.setMotorVoltage:
			return self.typeConvertor.IntToHex(parameters[0]) + self.typeConvertor.FloatToHex(parameters[1])
		elif comand == CommandsList.setPidParameters:
			return self.GetByteStringFromFloatList(parameters)
		elif comand == CommandsList.setRotatoinSpeed:
			return self.GetByteStringFromFloatList(parameters)
		elif comand == CommandsList.setMovementSpeed:
			return self.typeConvertor.FloatToHex(parameters[0]) + self.typeConvertor.FloatToHex(parameters[1]) + self.typeConvertor.FloatToHex(parameters[2])
		elif comand == CommandsList.addPointToStack:
			return self.typeConvertor.FloatToHex(parameters[0]) + self.typeConvertor.FloatToHex(parameters[1]) + self.typeConvertor.FloatToHex(parameters[2]) + self.typeConvertor.IntToHex(parameters[3])
		elif comand == CommandsList.setMovementParameters:
			return self.typeConvertor.FloatToHex(parameters[0]) + self.typeConvertor.FloatToHex(parameters[1]) + self.typeConvertor.FloatToHex(parameters[2]) + self.typeConvertor.FloatToHex(parameters[3]) + self.typeConvertor.FloatToHex(parameters[4])
		elif comand == CommandsList.setADCPinMode:
			return  self.typeConvertor.IntToHex(parameters[0]) + self.typeConvertor.IntToHex(parameters[1])
		elif comand == CommandsList.getADCPinState:
			return self.typeConvertor.IntToHex(parameters)
		elif comand == CommandsList.getDigitalPinState:
			return self.typeConvertor.IntToHex(parameters)
		elif comand == CommandsList.setOutputState:
			return self.typeConvertor.IntToHex(parameters)
		elif comand == CommandsList.getPinMode:
			return self.typeConvertor.IntToHex(parameters)
		elif comand == CommandsList.setPinModeExit:
			return self.typeConvertor.IntToHex(parameters[0]) + self.typeConvertor.IntToHex(parameters[1])
		elif comand == CommandsList.getDiscretePinState:
			return self.typeConvertor.IntToHex(parameters)
		elif comand == CommandsList.setDiscreteOutputState:
			return self.typeConvertor.IntToHex(parameters)
		elif comand == CommandsList.determineCurrentPinMode:
			return self.typeConvertor.IntToHex(parameters)
		elif comand == CommandsList.set12VState:
			return self.typeConvertor.IntToHex(parameters)
		elif comand == CommandsList.stopAllMotors:
			return self.typeConvertor.FloatToHex(parameters[0]) + self.typeConvertor.FloatToHex(parameters[1]) + self.typeConvertor.FloatToHex(parameters[2]) + self.typeConvertor.IntToHex(parameters[3])
		elif comand == CommandsList.switchOnVibrationTable:
			return self.typeConvertor.IntToHex(parameters)
		elif comand == CommandsList.moveWithCorrection:
			return self.typeConvertor.FloatToHex(parameters[0]) + self.typeConvertor.FloatToHex(parameters[1]) + self.typeConvertor.FloatToHex(parameters[2]) + self.typeConvertor.FloatToHex(parameters[3]) + self.typeConvertor.FloatToHex(parameters[4]) + self.typeConvertor.FloatToHex(parameters[5]) + self.typeConvertor.IntToHex(parameters[6])
		return ""

	def GetByteStringFormString(self, parameters):
			outputString = ""			
			for symbol in parameters:
				outputString = outputString + self.typeConvertor.IntToHex(ord(symbol))			
			return outputString

	def GetByteStringFromIntList(self, parameters):
		outputString = ""
		for parameter in parameters:
			if type(parameter) is not int:
				raise Exception('Parameter should be integer number.')
			outputString = outputString + self.typeConvertor.IntToHex(parameter)
		return outputString

	def GetByteStringFromFloatList(self, parameters):
			outputString = ""
			for parameter in parameters:
				outputString = outputString + self.typeConvertor.FloatToHex(parameter)
			return outputString

class CommandsList(object):
	echo = 0x01	#expected parameters: char[4] = 'ECHO'
	setCoordinates = 0x02	#expected parameters: float32[3]
	dutyCycle = 0x03	#expected parameters: int[1], float32[1]
	setDirectionBit = 0x04 #expected parameters: int[1]
	removeDirectionBit = 0x05	#expected parameters: int[1]
	setMotorVoltage = 0x06	#expected parameters: int[1], float32[1]
	setPidParameters = 0x08	#expected parameters: float32[3]
	setRotatoinSpeed = 0x09	#expected parameters: float32[4]
	switchOnKinematicCalculation = 0xB
	switchOffKinematicCalculation = 0xC 
	setMovementSpeed = 0xD #expected parameters: float32[3]
	switchOnTrajectoryRegulator = 0xE
	switchOffTrajectoryRegulator = 0xF
	cleanPointsStack = 0x10
	addPointToStack = 0x11 #expected parameters: float32[3], int[1]
	getStackState = 0x12
	getCurentCoordinates = 0x13
	getCurrentSpeed = 0x14
	setMovementParameters = 0x15	#expected parameters: float32[5]
	setADCPinMode = 0x16	#expected parameters: int[1], int[1]
	getADCPinState = 0x17	#expected parameters: int[1]
	getAllADCPinsStet = 0x18
	getDigitalPinState = 0x19	#expected parameters: int[1]
	getAllDigitalPinState = 0x1a
	setOutputState = 0x1b	#expected parameters: int[1]
	getPinMode = 0x1c	#expected parameters: int[1]
	setPinModeExit = 0x1d	#expected parameters: int[2]
	getDiscretePinState = 0x1e	#expected parameters: int[1]
	getAllDiscretePinStates = 0x1f
	setDiscreteOutputState = 0x20	#expected parameters: int[1]
	determineCurrentPinMode = 0x21	#expected parameters: int[1]
	set12VState = 0x22 #expected parameters: int[1]
	switchOffPid = 0x23
	switchOnPid = 0x24
	stopAllMotors = 0x29
	setCorectCoordinates = 0x25
	
	# TODO: playing field side
	# TODO: beginning of the competition sign
	# TODO: implement commands listed below	
	#getManipulatorState = 0x26
	#changeSuckerState = 0x27	#expected parameters: int[1]
	#uploadPuck = 0x28
	#unloadAllPucks = 0x29	#expected parameters: int[1]
	#changeFishingRodState = 0x30	#expected parameters: int[1]
	#changeFishingLatchState = 0x2A	#expected parameters: int[1]
	openCubeCollector = 0x2B
	closeCubeCollector = 0x2C
	setManipulatorAngle = 0x31 # expected parameter: flot[1]
	releaseCubeMoovers = 0x2D
	raiseCubeMoovers = 0x2E
	switchOnVibrationTable = 0x2F #expected parameters: int[1]
	switchOffVibrationTable = 0x30
	switchOffBelts = 0x33
	startGame = 0x34
	openCubeBorder = 0x2D
	closeCubeBorder = 0x2E
	isPointWasReached = 0x32	
	switchOnCollisionAvoidance = 0x33
	switchOffCollisionAvoidance = 0x34
	getManipulatorAngle = 0x35
	getDataIKSensors = 0x36
	getDataUSSensors = 0x37
	switchOffRobot = 0x38
	moveWithCorrection = 0x39 #expected parameters: float32[6], float32[1]
	openCubeManipulatorBigAngle = 0x3A
	openConeCrasher = 0x3B
	closeConeCrasher = 0x3C

