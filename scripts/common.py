#!/usr/bin/env python


class Sensor (object):
    def __init__ (self,class_name, topic_name ,data):
        self.class_name = class_name
        self.topic_name = topic_name
        self.data              = data

