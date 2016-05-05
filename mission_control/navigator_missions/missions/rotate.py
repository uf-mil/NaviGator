from txros import util
import numpy as np

@util.cancellableInlineCallbacks
def run(boat, params):
	print "Rotating " + params + " degrees"
	params = int(params)
	yield boat.move.yaw_left_deg(params).go()
