from txros import util

@util.cancellableInlineCallbacks
def run(boat, params):
    print "Moving right " + params + "m"
    params = int(params)
    yield boat.move.right(params).go()
    print "Done!"