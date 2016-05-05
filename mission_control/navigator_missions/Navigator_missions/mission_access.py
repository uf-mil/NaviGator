#!/usr/bin/env python

import signal
import traceback
import argparse
from twisted.internet import defer, reactor
import txros
from Navigator_missions import txboat
import missions

'''

The mission system is highly highly highly adapted from the one used on Sub8. There are some
differences on the higher levels but as far as how a pose is commanded and edited we use
PoseEditor just as the sub does.

Thanks Forrest for doing the hard work, and Jacob for figuring out how to do it with reactor

The justification for making this a class rather than only able to run from the command line
is that one day we may want to call missions from other scripts or possibly build missions
and excecute in real time. So we make a mission object that lays all the groundwork connecting
to PoseEditor and txros and other junk then calls other submission scripts.

Right now it only takes in command line arguments

Ex. rosrun navigator_missions mission_access.py forward 5 -> move forward 5 meters
Ex. rosrun navigator_missions mission_access.py rotate 90 -> rotate 90 degrees CC
Ex. rosrun navigator_missions mission_access.py xyz_move_relative [1,5] -> move one meter in the x direction and 5 in the y

'''

class Mission(object):

    def __init__(self):

        self.boat = None

        reactor.callWhenRunning(self.start)
        reactor.run()

    @txros.util.cancellableInlineCallbacks
    def main(self):
        try:
            nh_args = yield txros.NodeHandle.from_argv_with_remaining('navigator_mission')
            nh, args = nh_args
            self.boat = yield txboat.get_boat(nh)
            yield txros.util.wall_sleep(1.0)

            available_missions = [mission_name for mission_name in dir(missions) if not mission_name.startswith('_')]

            usage_msg = ("Input the name of the mission you would like to run.\nExamples: " +
                     "\n\n\trosrun sub8_missions mission stop\n\trosrun sub8_missions mission forward_1_m")
            desc_msg = "-- Mr. Mission Manager --"
            parser = argparse.ArgumentParser(usage=usage_msg, description=desc_msg)
            parser.add_argument(dest='mission_name',
                                help="The name of the mission you'd like to run (ex: stop)")

            parser.add_argument('--test', dest='test', action='store_true',
                                help="Set this flag if you'd like to ignore motions")

            mission = getattr(missions, mission_name)
            script = parser.parse_args(args[1:2])
            params = parser.parse_args(args[2:])

            mission_name = script.mission_name
            params = params.mission_name
            assert mission_name in available_missions, "'{}' is not an available mission; we have: {}".format(
                mission_name, ', '.join(available_missions)
            )

            mission = getattr(missions, mission_name)

            yield mission.run(self.boat, params)
            yield txros.util.wall_sleep(1.0)

        except Exception:
            traceback.print_exc()

        finally:
            print 'Finishing mission execution'
            reactor.stop()

    def start(self):
        # Handle ctrl+C
        signal.signal(signal.SIGINT, lambda signum, frame: reactor.callFromThread(task.cancel))
        # Handle errors in main
        task = self.main().addErrback(lambda fail: fail.trap(defer.CancelledError))

if __name__ == '__main__':

    m = Mission()
