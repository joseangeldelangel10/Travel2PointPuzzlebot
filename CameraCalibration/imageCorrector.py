import cv2 as cv
import numpy as np
import json

f = open("perspactiveCorrectionData.json")
data = json.load(f)

left_tranformation_matrix = data["leftTransformationMat"]
right_tranformation_matrix = data["rightTransformationMat"]
transformation_size = data["transformationSize"]

left_tranformation_matrix = np.array(left_tranformation_matrix)
right_tranformation_matrix = np.array(right_tranformation_matrix)
left_tranformation_matrix = np.reshape(left_tranformation_matrix, (3,3))
right_tranformation_matrix = np.reshape(right_tranformation_matrix, (3,3))


img = cv.imread("/home/jose/Documents/6toSemestre/chrisAct1/CameraCalibration/stop.png")
im_height = img.shape[0]
im_width = img.shape[1]

one_third_imW = int(im_width*(1.0/3.0))
two_thirds_imW = int(im_width*(2.0/3.0))
one_third_imH = int(im_height*(1.0/3.0))
two_thirds_imH = int(im_height*(2.0/3.0))

left_side = img[:,:one_third_imW-1]
right_side = img[:, two_thirds_imW-1:]

result = cv.warpPerspective(left_side, left_tranformation_matrix, (transformation_size[1], transformation_size[0]) , flags= cv.INTER_LINEAR)
result2 = cv.warpPerspective(right_side, right_tranformation_matrix, (transformation_size[1], transformation_size[0]) , flags= cv.INTER_LINEAR)
cv.imshow("original", img)
cv.imshow("result", result)
cv.imshow("result2", result2)
cv.waitKey(0)