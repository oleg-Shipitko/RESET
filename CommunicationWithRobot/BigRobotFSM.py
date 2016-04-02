import packetBuilder

State = type ("State", (object,), {})

class CommandsList(object):

	def GoToPoint(x, y, alpha):
		#####

	def PickTower(activeSide, floor):
		#####

	def CheckTower(activeSide):
		#####

	def MoveTrack(positionNumber):
		#####

	def PickCubes(floor):	
		#####	

	def ThrowTower(activeSide):
		#####

	def ThrowCubes(cubesNumber):
		#####

	def TurnOnGrab():
		#####		

	def TurnOffGrab():
		#####	


class TakeTower_1_1(State):
	def Execute(self):
		CommandsList = CommandsList()
		CommandsList.PickTower("right", 2)

class MoveToTower_1_2(State):
	def Execute(self):
		CommandsList = CommandsList()
		CommandsList.GoToPoint()
		CommandsList.MoveTrack()

