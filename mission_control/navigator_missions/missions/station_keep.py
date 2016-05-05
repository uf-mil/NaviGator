from txros import util

@util.cancellableInlineCallbacks
def run(boat, params):
    print "Station Keeping"
    yield boat.station_hold()
    print "Done!"
