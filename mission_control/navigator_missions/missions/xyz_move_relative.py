from txros import util
import numpy as np

@util.cancellableInlineCallbacks
def run(boat, params):
	x = params.replace("[", "").replace("]", "").split(",")[0]
	y = params.replace("[", "").replace("]", "").split(",")[1]
	rel_move = np.array(([float(x),float(y), 0]))
	yield boat.move.relative(rel_move).go()
