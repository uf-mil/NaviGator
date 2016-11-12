#!/usr/bin/env python
import txros
from navigator_tools import fprint
from navigator_tools import MissingPerceptionObject


@txros.util.cancellableInlineCallbacks
def main(navigator):
    nh = navigator.nh

    fprint("{} running".format(__name__), msg_color='red')

    obj = yield navigator.database_query("scan_the_code")
    fprint("{} stopped running, raising exception".format(__name__), msg_color='red')
    if len(obj.objects) == 0:
        raise MissingPerceptionObject("scan_the_code")

    yield nh.sleep(3)