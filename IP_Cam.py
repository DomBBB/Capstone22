import datetime
import time
from collections import OrderedDict

import mysql.connector

import cv2

import dlib

import imutils
from imutils.video import FPS, VideoStream

import numpy as np

from scipy.spatial import distance as dist


class CentroidTracker:
    """To track people we are using an already implemented Centroid Tracker,
    taken from: https://github.com/Neoanarika/object-tracking-detection/blob/
    master/centroidtracker.py and added a maximum distance like it has been
    done in https://gist.github.com/ouujj/30ad855f8b0449a7b4594d82f45dd214"""
    def __init__(self, maxDisappeared=50, maxDistance=50):
        # initialize the next unique object ID along with two ordered
        # dictionaries used to keep track of mapping a given object
        # ID to its centroid and number of consecutive frames it has
        # been marked as "disappeared", respectively
        self.nextObjectID = 0
        self.objects = OrderedDict()
        self.disappeared = OrderedDict()

        # store the number of maximum consecutive frames a given
        # object is allowed to be marked as "disappeared" until we
        # need to deregister the object from tracking
        self.maxDisappeared = maxDisappeared

        # store the maximum distance between centroids to associate
        # an object -- if the distance is larger than this maximum
        # distance we'll start to mark the object as "disappeared"
        self.maxDistance = maxDistance

    def register(self, centroid):
        # when registering an object we use the next available object
        # ID to store the centroid
        self.objects[self.nextObjectID] = centroid
        self.disappeared[self.nextObjectID] = 0
        self.nextObjectID += 1

    def deregister(self, objectID):
        # to deregister an object ID we delete the object ID from
        # both of our respective dictionaries
        del self.objects[objectID]
        del self.disappeared[objectID]

    def update(self, rects):
        # check to see if the list of input bounding box rectangles
        # is empty
        if len(rects) == 0:
            # loop over any existing tracked objects and mark them
            # as disappeared
            for objectID in self.disappeared.keys():
                self.disappeared[objectID] += 1

                # if we have reached a maximum number of consecutive
                # frames where a given object has been marked as
                # missing, deregister it
                if self.disappeared[objectID] > self.maxDisappeared:
                    self.deregister(objectID)

            # return early as there are no centroids or tracking info
            # to update
            return self.objects

        inputCentroids = np.zeros((len(rects), 2), dtype="int")

        # loop over the bounding box rectangles
        for (i, (startX, startY, endX, endY)) in enumerate(rects):
            # use the bounding box coordinates to derive the centroid
            cX = int((startX + endX) / 2.0)
            cY = int((startY + endY) / 2.0)
            inputCentroids[i] = (cX, cY)

        if len(self.objects) == 0:
            for i in range(0, len(inputCentroids)):
                self.register(inputCentroids[i])
        else:
            # grab the set of object IDs and corresponding centroids
            objectIDs = list(self.objects.keys())
            objectCentroids = list(self.objects.values())

            # compute the distance between each pair of object
            # centroids and input centroids, respectively -- our
            # goal will be to match an input centroid to an existing
            # object centroid
            D = dist.cdist(np.array(objectCentroids), inputCentroids)

            # in order to perform this matching we must (1) find the
            # smallest value in each row and then (2) sort the row
            # indexes based on their minimum values so that the row
            # with the smallest value is at the *front* of the index
            # list
            rows = D.min(axis=1).argsort()

            # next, we perform a similar process on the columns by
            # finding the smallest value in each column and then
            # sorting using the previously computed row index list
            cols = D.argmin(axis=1)[rows]
            usedRows = set()
            usedCols = set()

            # loop over the combination of the (row, column) index
            # tuples
            for (row, col) in zip(rows, cols):
                # if we have already examined either the row or
                # column value before, ignore it
                # val
                if row in usedRows or col in usedCols:
                    continue

                # if the distance between centroids is greater than
                # the maximum distance, do not associate the two
                # centroids to the same object
                if D[row, col] > self.maxDistance:
                    continue

                # otherwise, grab the object ID for the current row,
                # set its new centroid, and reset the disappeared
                # counter
                objectID = objectIDs[row]
                self.objects[objectID] = inputCentroids[col]
                self.disappeared[objectID] = 0

                # indicate that we have examined each of the row and
                # column indexes, respectively
                usedRows.add(row)
                usedCols.add(col)

            unusedRows = set(range(0, D.shape[0])).difference(usedRows)
            unusedCols = set(range(0, D.shape[1])).difference(usedCols)

            if D.shape[0] >= D.shape[1]:
                # loop over the unused row indexes
                for row in unusedRows:
                    # grab the object ID for the corresponding row
                    # index and increment the disappeared counter
                    objectID = objectIDs[row]
                    self.disappeared[objectID] += 1

                    # check to see if the number of consecutive
                    # frames the object has been marked "disappeared"
                    # for warrants deregistering the object
                    if self.disappeared[objectID] > self.maxDisappeared:
                        self.deregister(objectID)
            else:
                for col in unusedCols:
                    self.register(inputCentroids[col])

        return self.objects


class TrackableObject:
    """This class instantiates new centroid tracked objects to store whether
    they were already counted."""
    def __init__(self, objectID, centroid):
        # store the object ID, then initialize a list of centroids
        # using the current centroid
        self.objectID = objectID
        self.centroids = [centroid]

        # initialize a boolean used to indicate if the object has
        # already been counted or not
        self.counted = False


def run():
    """This function runs the main code to detect people entering / exiting.
    Inspired and adapted from https://pyimagesearch.com/2018/08/13/opencv-
    people-counter/"""
    # parameters to adjust the model and configure the video stream
    params = {
        # pretrained model from https://github.com/amolikvivian/Caffe-SSD-
        # Object-Detection/tree/master/Object%20Detection%20Caffe/Caffe
        "prototxt": "mobilenet_ssd/SSD_MobileNet.prototxt",
        "model": "mobilenet_ssd/SSD_MobileNet.caffemodel",
        # list of class labels the MobileNet SSD was trained on
        "classes": ["background", "aeroplane", "bicycle", "bird", "boat",
                    "bottle", "bus", "car", "cat", "chair", "cow",
                    "diningtable", "dog", "horse", "motorbike", "person",
                    "pottedplant", "sheep", "sofa", "train", "tvmonitor"],
        # url of ip camera
        #
        "url": "rtsp://admin12345:root12345@172.20.10.9/stream1",
        "confidence": 0.2,
        "skip_frames": 30}

    # load the serialized Caffe model
    net = cv2.dnn.readNetFromCaffe(params["prototxt"], params["model"])

    # start the video stream
    print("[INFO] Starting the live stream..")
    vs = VideoStream(params["url"]).start()
    time.sleep(2.0)

    # initialize frame size
    W = None
    H = None

    # instantiate the centroid tracker
    ct = CentroidTracker(maxDisappeared=40, maxDistance=50)

    # list to store each of the dlib correlation trackers
    trackers = []
    # dictionary to map each unique object ID to a TrackableObject
    trackableObjects = {}

    # start the frames per second throughput estimator
    fps = FPS().start()
    totalFrames = 0

    # initalize variables to store our results
    totalDown = 0
    totalUp = 0
    x = []
    empty = []
    empty1 = []

    # loop over video stream frames
    while True:
        frame = vs.read()
        frame = frame

        # resize for faster processing, convert from BGR to RGB (for dlib)
        frame = imutils.resize(frame, width=500)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # set frame size
        if W is None or H is None:
            (H, W) = frame.shape[:2]

        # initialize status and list of box rectangles (detected or tracked)
        status = "Waiting"
        rects = []

        # runs an object detection method every few frames
        if totalFrames % params["skip_frames"] == 0:
            # initializes the new set of object trackers
            status = "Detecting"
            trackers = []

            # convert frame to blob and pass to the net
            blob = cv2.dnn.blobFromImage(frame, 0.007843, (W, H), 127.5)
            net.setInput(blob)
            detections = net.forward()

            # loop over all detections
            for i in np.arange(0, detections.shape[2]):

                # only count detections over the confidence threshold
                confidence = detections[0, 0, i, 2]
                if confidence > params["confidence"]:
                    # only count detections with class label person
                    idx = int(detections[0, 0, i, 1])
                    if params["classes"][idx] != "person":
                        continue

                    # construct the dlib rectangle for the (x, y)-coordinates
                    # and start the dlib correlation tracker
                    box = detections[0, 0, i, 3:7] * np.array([W, H, W, H])
                    (startX, startY, endX, endY) = box.astype("int")
                    tracker = dlib.correlation_tracker()
                    rect = dlib.rectangle(startX, startY, endX, endY)
                    tracker.start_track(rgb, rect)

                    # append tracker to trackers list in order to use it during
                    # skip frames
                    trackers.append(tracker)

        # if not object detection we use our object tracker to get higher frame
        # processing
        else:
            # loop over all trackers
            for tracker in trackers:
                # update the tracker, retrieve the new position
                status = "Tracking"
                tracker.update(rgb)
                pos = tracker.get_position()

                # add the box rectangle coordinates to the rectangles list
                startX = int(pos.left())
                startY = int(pos.top())
                endX = int(pos.right())
                endY = int(pos.bottom())
                rects.append((startX, startY, endX, endY))

        # show a line in the center of the frame, if an object crosses it we
        # count this as either an entry or an exit
        cv2.line(frame, (0, H // 2), (W, H // 2), (0, 0, 0), 3)
        cv2.putText(frame, "Entrance / Exit", (10, H - ((i * 20) + 200)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        # associate old object centroid with new object centroid
        objects = ct.update(rects)

        # loop over all tracked objects
        for (objectID, centroid) in objects.items():
            # If there is no trackable object for the current object ID create
            # a new one, otherwise there exists a trackable object we can
            # utilize to determine the walking direction
            to = trackableObjects.get(objectID, None)
            if to is None:
                to = TrackableObject(objectID, centroid)
            else:
                # we can calculate the direction by comparing the y-coordinate
                # of the current centroid with the mean of the previous one
                y = [c[1] for c in to.centroids]
                direction = centroid[1] - np.mean(y)
                to.centroids.append(centroid)

                # check if we already counted the object
                if not to.counted:
                    # counts the people entering
                    if direction < 0 and centroid[1] < H // 2:
                        totalDown += 1
                        empty.append(totalDown)
                        to.counted = True

                    # counts the people exiting
                    elif direction > 0 and centroid[1] > H // 2:
                        totalUp += 1
                        empty1.append(totalUp)
                        to.counted = True

                    # sum of total people inside
                    x = []
                    x.append(len(empty)-len(empty1))

            # store trackable object in dictionary
            trackableObjects[objectID] = to

            # draw the object centroid and ID on the output frame
            text = "ID {}".format(objectID)
            cv2.putText(frame, text, (centroid[0] - 10, centroid[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            cv2.circle(frame, (centroid[0], centroid[1]), 4, (255, 255, 255),
                       -1)

        # construct information to display on the output frame
        info = [("Exit", totalUp),
                ("Enter", totalDown),
                ("Status", status)]

        info2 = [("Total people inside", x)]

        # logs occupancy all 3000 frames
        if totalFrames % 3000 == 0:
            export_data = ",".join([str(totalDown), str(totalUp), str(x)])
            export_time = datetime.datetime.now().strftime("%m/%d/%Y %H:%M:%S")
            with open("Log.csv", "a") as f:
                f.write(export_time + "," + export_data + "\n")
            cnx = mysql.connector.connect(user='sql11489402', password='8Dl4twiWj3', host='sql11.freemysqlhosting.net', database='sql11489402')
            cursor = cnx.cursor()
            query = ("REPLACE INTO sensors (Sensor, Value) VALUES ('IP_Cam', %s)" % export_data)
            cursor.execute(query)
            cursor.close()
            cnx.close()

        # display output
        for (i, (k, v)) in enumerate(info):
            text = "{}: {}".format(k, v)
            cv2.putText(frame, text, (10, H - ((i * 20) + 20)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)

        for (i, (k, v)) in enumerate(info2):
            text = "{}: {}".format(k, v)
            cv2.putText(frame, text, (265, H - ((i * 20) + 60)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # show output frame
        cv2.imshow("Real-Time Monitoring Window", frame)
        key = cv2.waitKey(1) & 0xFF

        # pressing "q" breaks the loop
        if key == ord("q"):
            break

        # processed frames and fps for debug purposes
        totalFrames += 1
        fps.update()

    # stop timer and display fps information
    fps.stop()
    print("[INFO] elapsed time: {:.2f}".format(fps.elapsed()))
    print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))

    # close all open windows
    cv2.destroyAllWindows()


run()
