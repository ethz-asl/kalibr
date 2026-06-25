import aslam_cv as acv
import cv2
import os
import numpy as np
import random
import xml.etree.ElementTree as ET


class CvatImageDatasetReaderIterator(object):
  def __init__(self, dataset, shuffle):
    self.dataset = dataset
    self.idx = 0
    self.idxs = list(range(dataset.numImages()))
    if shuffle:
      self.idxs = random.shuffle(self.idxs)

  def __iter__(self):
    return self

  def next(self):
    # required for python 2.x compatibility
    idx = self.idx
    self.idx += 1
    if self.idx >= len(self.idxs):
      raise StopIteration()
    return self.dataset.getImage(self.idxs[idx])

  def __next__(self):
    idx = self.idx
    self.idx += 1
    if self.idx >= len(self.idxs):
      raise StopIteration()
    return self.dataset.getImage(self.idxs[idx])


class CvatAnnotation(object):
  def __init__(self, points_data, image_data):
    self.id = int(image_data.attrib["id"])
    self.image_name = image_data.attrib["name"]
    self.width = int(image_data.attrib["width"])
    self.height = int(image_data.attrib["height"])
    self.points = list()
    for p in points_data.attrib["points"].split(';'):
      p1, p2 = p.split(',')
      self.points.append(np.array([float(p1), float(p2)]))


class CvatImageDatasetReader(object):
  def __init__(self, annotations_file, label=None):
    self.annotations_file = annotations_file
    self.topic = label

    tree = ET.parse(annotations_file)
    root = tree.getroot()
    assert(root.tag == 'annotations')

    self.annotations = list()
    self.annotations_by_id = dict()
    for child in root:
      if child.tag != "image":
        continue
      annotation_tag = None
      for e in child:
        if e.tag == 'points':
          if label is None or e.attrib['label'] == label:
            annotation_tag = e
            break

      if annotation_tag is not None:
        annotation = CvatAnnotation(annotation_tag, child)
        self.annotations.append(annotation)
        self.annotations_by_id[annotation.id] = annotation

  def __iter__(self):
    # Reset the bag reading
    return self.readDataset()

  def readDataset(self):
    return CvatImageDatasetReaderIterator(self, shuffle=False)

  def readDatasetShuffle(self):
    return CvatImageDatasetReaderIterator(self, shuffle=True)

  def numImages(self):
    return len(self.annotations)

  def getImage(self, idx):
    image_path = os.path.join(os.path.dirname(self.annotations_file),
                              self.annotations[idx].image_name)
    timestamp = acv.Time(self.annotations[idx].id, 0)
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    # return ID instead of timestamp
    return timestamp, img


class CvatImageDatasetDetector(object):
  def __init__(self, dataset, grid, cameraGeometry, showCorners=False):
    self.dataset = dataset
    self.grid = grid
    self.cameraGeometry = cameraGeometry
    self.showCorners = showCorners

    if self.showCorners:
      cv2.namedWindow("CVAT: Corner detections")
      cv2.resizeWindow("CVAT: Corner detections", 640, 480)

  def target(self):
    return self.grid

  def findTargetNoTransformation(self, timestamp, image):
    observation = acv.GridCalibrationTargetObservation(self.grid)
    observation.setTime(timestamp)
    observation.setImage(image)

    id = timestamp.sec
    if id not in self.dataset.annotations_by_id:
      return False, observation

    annotation = self.dataset.annotations_by_id[id]
    assert isinstance(annotation, CvatAnnotation)

    if self.showCorners:
      img_copy = np.copy(image)

    w = annotation.width
    h = annotation.height
    num_valid = 0
    for i, p in enumerate(annotation.points):
      if all(np.isfinite(p)) and 0 <= p[0] < w and 0 <= p[1] < h:
        observation.updateImagePoint(i, p)
        num_valid += 1
        if self.showCorners:
          cv2.circle(img_copy, (int(p[0]), int(p[1])), 3, color=(0, 0, 255), thickness=1)
          cv2.circle(img_copy, (int(p[0]), int(p[1])), 4, color=(255, 255, 255), thickness=1)

    if self.showCorners:
      cv2.imshow("CVAT: Corner detections", img_copy)
      cv2.waitKey(1)

    return num_valid >= 4, observation

  def findTarget(self, timestamp, image):
    success, observation = self.findTargetNoTransformation(timestamp, image)
    if not success:
      return success, observation

    success, tf = self.cameraGeometry.estimateTransformation(observation)
    if success:
      observation.set_T_t_c(tf)

    return success, observation