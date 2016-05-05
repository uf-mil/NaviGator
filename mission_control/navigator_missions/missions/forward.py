from txros import util

@util.cancellableInlineCallbacks
def run(boat, params):
    print "Moving forward " + params + "m"
    params = int(params)
    yield boat.move.forward(params).go()
    print "Done!"
