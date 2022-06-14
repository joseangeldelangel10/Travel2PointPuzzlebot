import cv2 as cv
import numpy as np


def array2string(array):
        if len(array) >= 1: 
            result = "\'  ,  \'".join(array)
            result = "[\'" + result
            result += "\']"
            return result
        else:
            return "[]"
 
img = cv.imread("/home/jose/Documents/6toSemestre/chrisAct1/CameraCalibration/3.jpg")
 
im_height = img.shape[0]
im_width = img.shape[1]
print("image shape is: " + str(img.shape) )

one_third_imH = int(im_height*(1/3))
two_thirds_imH = int(im_height*(2/3))
one_third_imW = int(im_width*(1/3))
two_thirds_imW = int(im_width*(2/3))
print("cuts are on: {one_third_imH} , {two_thirds_imH}, {one_third_imW}, {two_thirds_imW}".format(one_third_imH=one_third_imH, two_thirds_imH=two_thirds_imH, one_third_imW=one_third_imW, two_thirds_imW=two_thirds_imW) )

height_elongation_ratio = 1.2
width_elongation_ratio = 0.8
height_elongation_ratio_diff = height_elongation_ratio - 1
width_elongation_ratio_diff = width_elongation_ratio - 1

imgCopy1 = img.copy()

# =============================== FIRST TRAPEZIUM  =========================
# Locate points of the documents
# or object which you want to transform
pts1_1 = np.float32([[one_third_imW, two_thirds_imH],
                    [two_thirds_imW, two_thirds_imH],
                    [1, im_height],
                    [im_width, im_height]]) 

pts1_1 = pts1_1 -1

cv.rectangle(imgCopy1, np.uint(pts1_1[0,:]), np.uint(pts1_1[0,:]), (0,0,255), 4)
cv.rectangle(imgCopy1, np.uint(pts1_1[1,:]), np.uint(pts1_1[1,:]), (0,0,255), 4)
cv.rectangle(imgCopy1, np.uint(pts1_1[2,:]), np.uint(pts1_1[2,:]), (0,0,255), 4)
cv.rectangle(imgCopy1, np.uint(pts1_1[3,:]), np.uint(pts1_1[3,:]), (0,0,255), 4)

pts1_2 = np.float32([ [one_third_imW - int(one_third_imW*width_elongation_ratio_diff/2) , two_thirds_imH-int(one_third_imH*(height_elongation_ratio_diff))],
                    [two_thirds_imW + int(one_third_imW*width_elongation_ratio_diff/2), two_thirds_imH-int(one_third_imH*(height_elongation_ratio_diff))],
                    [1, im_height],
                    [im_width, im_height]]) 

pts1_2 = pts1_2 -1

cv.rectangle(imgCopy1, np.uint(pts1_2[0,:]), np.uint(pts1_2[0,:]), (0,255,0), 4)
cv.rectangle(imgCopy1, np.uint(pts1_2[1,:]), np.uint(pts1_2[1,:]), (0,255,0), 4)
cv.rectangle(imgCopy1, np.uint(pts1_2[2,:]), np.uint(pts1_2[2,:]), (0,255,0), 4)
cv.rectangle(imgCopy1, np.uint(pts1_2[3,:]), np.uint(pts1_2[3,:]), (0,255,0), 4)

# =============================== SECOND TRAPEZIUM  =========================
# Locate points of the documents
# or object which you want to transform
pts2_1 = np.float32([[two_thirds_imW, two_thirds_imH],
                    [two_thirds_imW, one_third_imH],
                    [im_width, im_height],
                    [im_width, 1]]) 

pts2_1 = pts2_1 -1

cv.rectangle(imgCopy1, np.uint(pts2_1[0,:]), np.uint(pts2_1[0,:]), (0,0,255), 4)
cv.rectangle(imgCopy1, np.uint(pts2_1[1,:]), np.uint(pts2_1[1,:]), (0,0,255), 4)
cv.rectangle(imgCopy1, np.uint(pts2_1[2,:]), np.uint(pts2_1[2,:]), (0,0,255), 4)
cv.rectangle(imgCopy1, np.uint(pts2_1[3,:]), np.uint(pts2_1[3,:]), (0,0,255), 4)

pts2_2 = np.float32([[two_thirds_imW - int(one_third_imW*height_elongation_ratio_diff), two_thirds_imH + int(one_third_imH*width_elongation_ratio_diff/2)],
                    [two_thirds_imW - int(one_third_imW*height_elongation_ratio_diff), one_third_imH - int(one_third_imH*width_elongation_ratio_diff/2)],
                    [im_width, im_height],
                    [im_width, 1]]) 

pts2_2 = pts2_2 -1

cv.rectangle(imgCopy1, np.uint(pts2_2[0,:]), np.uint(pts2_2[0,:]), (0,255,0), 4)
cv.rectangle(imgCopy1, np.uint(pts2_2[1,:]), np.uint(pts2_2[1,:]), (0,255,0), 4)
cv.rectangle(imgCopy1, np.uint(pts2_2[2,:]), np.uint(pts2_2[2,:]), (0,255,0), 4)
cv.rectangle(imgCopy1, np.uint(pts2_2[3,:]), np.uint(pts2_2[3,:]), (0,255,0), 4)

# =============================== THIRD TRAPEZIUM  =========================
# Locate points of the documents
# or object which you want to transform
pts3_1 = np.float32([[two_thirds_imW, one_third_imH],
                    [one_third_imW, one_third_imH],
                    [im_width, 1],
                    [1, 1]]) 

pts3_1 = pts3_1 -1

cv.rectangle(imgCopy1, np.uint(pts3_1[0,:]), np.uint(pts3_1[0,:]), (255,0,0), 4)
cv.rectangle(imgCopy1, np.uint(pts3_1[1,:]), np.uint(pts3_1[1,:]), (255,0,0), 4)
cv.rectangle(imgCopy1, np.uint(pts3_1[2,:]), np.uint(pts3_1[2,:]), (255,0,0), 4)
cv.rectangle(imgCopy1, np.uint(pts3_1[3,:]), np.uint(pts3_1[3,:]), (255,0,0), 4)

pts3_2 = np.float32([[two_thirds_imW + int(one_third_imW*width_elongation_ratio_diff/2), one_third_imH + int(one_third_imH*height_elongation_ratio_diff)],
                    [one_third_imW - int(one_third_imW*width_elongation_ratio_diff/2) , one_third_imH + int(one_third_imH*height_elongation_ratio_diff)],
                    [im_width, 1],
                    [1, 1]])  

pts3_2 = pts3_2 -1

cv.rectangle(imgCopy1, np.uint(pts3_2[0,:]), np.uint(pts3_2[0,:]), (255,0,255), 4)
cv.rectangle(imgCopy1, np.uint(pts3_2[1,:]), np.uint(pts3_2[1,:]), (255,0,255), 4)
cv.rectangle(imgCopy1, np.uint(pts3_2[2,:]), np.uint(pts3_2[2,:]), (255,0,255), 4)
cv.rectangle(imgCopy1, np.uint(pts3_2[3,:]), np.uint(pts3_2[3,:]), (255,0,255), 4)

# =============================== THIRD TRAPEZIUM  =========================
# Locate points of the documents
# or object which you want to transform
pts4_1 = np.float32([[one_third_imW, one_third_imH],
                    [one_third_imW, two_thirds_imH],
                    [1, 1],
                    [1, im_height]]) 

pts4_1 = pts4_1 -1

cv.rectangle(imgCopy1, np.uint(pts4_1[0,:]), np.uint(pts4_1[0,:]), (255,0,0), 4)
cv.rectangle(imgCopy1, np.uint(pts4_1[1,:]), np.uint(pts4_1[1,:]), (255,0,0), 4)
cv.rectangle(imgCopy1, np.uint(pts4_1[2,:]), np.uint(pts4_1[2,:]), (255,0,0), 4)
cv.rectangle(imgCopy1, np.uint(pts4_1[3,:]), np.uint(pts4_1[3,:]), (255,0,0), 4)

pts4_2 = np.float32([[one_third_imW + int(one_third_imW*height_elongation_ratio_diff), one_third_imH - int(one_third_imH*width_elongation_ratio_diff/2)],
                    [one_third_imW + int(one_third_imW*height_elongation_ratio_diff), two_thirds_imH + int(one_third_imH*width_elongation_ratio_diff/2)],
                    [1, 1],
                    [1, im_height]])   

pts4_2 = pts4_2 -1

cv.rectangle(imgCopy1, np.uint(pts4_2[0,:]), np.uint(pts4_2[0,:]), (255,0,255), 4)
cv.rectangle(imgCopy1, np.uint(pts4_2[1,:]), np.uint(pts4_2[1,:]), (255,0,255), 4)
cv.rectangle(imgCopy1, np.uint(pts4_2[2,:]), np.uint(pts4_2[2,:]), (255,0,255), 4)
cv.rectangle(imgCopy1, np.uint(pts4_2[3,:]), np.uint(pts4_2[3,:]), (255,0,255), 4)

     
# Apply Perspective Transform Algorithm
matrix1 = cv.getPerspectiveTransform(pts1_1, pts1_2)
print("matrix1 is: ")
print(type(matrix1))
matrix2 = cv.getPerspectiveTransform(pts2_1, pts2_2)
matrix3 = cv.getPerspectiveTransform(pts3_1, pts3_2)
matrix4 = cv.getPerspectiveTransform(pts4_1, pts4_2)
first_trap_frame = img[two_thirds_imH-1:,:]
second_trap_frame = img[:,two_thirds_imW-1:]
third_trap_frame = img[:one_third_imH-1,:]
fourth_trap_frame = img[:,:one_third_imW-1]
result = cv.warpPerspective(first_trap_frame, matrix1, ( im_width + int(one_third_imW*width_elongation_ratio_diff), one_third_imH + int(one_third_imH*height_elongation_ratio_diff)) , flags= cv.INTER_LINEAR)
result2 = cv.warpPerspective(second_trap_frame, matrix2, ( one_third_imW + int(one_third_imW*height_elongation_ratio_diff), im_height + int(one_third_imH*width_elongation_ratio_diff)) , flags= cv.INTER_LINEAR)
result3 = cv.warpPerspective(third_trap_frame, matrix3, ( im_width + int(one_third_imW*width_elongation_ratio_diff), one_third_imH + int(one_third_imH*height_elongation_ratio_diff)) , flags= cv.INTER_LINEAR)
result4 = cv.warpPerspective(fourth_trap_frame, matrix4, ( one_third_imW + int(one_third_imW*height_elongation_ratio_diff), im_height + int(one_third_imH*width_elongation_ratio_diff)) , flags= cv.INTER_LINEAR)
    
# Wrap the transformed image
#cv.imshow("original with points 1", imgCopy1)
cv.imshow('original', img) # Initial Capture
#cv.imshow('trapesium', first_trap_frame) # Initial Capture
cv.imshow('result', result) # Transformed Capture
cv.imshow('result2', result2) # Transformed Capture
cv.imshow('result3', result3) # Transformed Capture
cv.imshow('result4', result4) # Transformed Capture

with open('perspactiveCorrectionData.json', 'w') as f:
    f.write("{" + "\n")
    f.write("\"transformationSize\":" + str([ result2.shape[0], result2.shape[1] ]) + ", \n")
    #non_distorted_image_indexes = [ [1/3,1/3],[2/3,1/3], [1/3,2/3],[2/3,2/3]]
    #f.write("\"nonDistortedImageIndexes\": " + str(non_distorted_image_indexes) + ", \n")
    tepm4 = np.reshape(matrix4, (9,))
    #tepm4 = np.where(np.logical_and(tepm4 < 0.001, tepm4 > -0.001), 0, tepm4)
    left_transformation_mat = str(tepm4.tolist())
    f.write("\"leftTransformationMat\": " + left_transformation_mat + ",\n")
    tepm2 = np.reshape(matrix2, (9,))
    #tepm2 = np.where(np.logical_and(tepm2 < 0.001, tepm2 > -0.001), 0, tepm2)
    right_transformation_mat = str(tepm2.tolist())
    f.write("\"rightTransformationMat\": " + right_transformation_mat + "\n")
    f.write("}")
 
cv.waitKey(0)