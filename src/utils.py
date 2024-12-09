#!/usr/bin/env python

from colorama import Fore, Style

import smach


def custom_loginfo(msg):
    print(
        Fore.GREEN
        + Style.BRIGHT
        + "[sm:info] "
        + Style.NORMAL
        + str(msg)
        + Style.RESET_ALL
    )


def custom_logwarn(msg):
    print(
        Fore.YELLOW
        + Style.BRIGHT
        + "[sm:warn] "
        + Style.NORMAL
        + str(msg)
        + Style.RESET_ALL
    )


def custom_logdebug(msg):
    pass


def custom_logerr(msg):
    print(
        Fore.RED
        + Style.BRIGHT
        + "[sm:err ] "
        + Style.NORMAL
        + str(msg)
        + Style.RESET_ALL
    )


def set_custom_loggers():
    smach.set_loggers(custom_loginfo, custom_logwarn, custom_logdebug, custom_logerr)
