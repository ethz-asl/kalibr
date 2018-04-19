#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2 as cv
import numpy as np
import os
import sys


def undistorted(folder_path_input, file_path_input, output_folder_input, mat_k, vec_d):
    image_distorted = cv.imread(folder_path_input + file_path_input)
    image_undistorted = image_distorted.copy()

    mat_k_new = mat_k
    cv.fisheye.undistortImage(image_distorted, mat_k, vec_d, image_undistorted, mat_k_new)

    if not os.path.exists(output_folder_input):
        os.makedirs(output_folder_input)
    cv.imwrite(output_folder_input + file_path_input, image_undistorted)


def get_all_file_path(file_dir):
    list_path = []
    for root, dirs, files in os.walk(file_dir):
        for file_name in files:
            list_path.append(file_name)
    return list_path


def write_original_xml(file_xml_path, height, width, k_matrix_input, d_vector_input):
    folder_path = os.path.dirname(file_xml_path)
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)

    fs_xml = cv.FileStorage(file_xml_path, cv.FILE_STORAGE_WRITE)
    fs_xml.write('image_width', width)
    fs_xml.write('image_height', height)

    fs_xml.write('camera_matrix', k_matrix_input)
    k3 = np.array([0])
    d_vector_input = np.squeeze(np.asarray(d_vector_input))
    d_vector_final = np.r_[d_vector_input, k3]
    fs_xml.write('distortion_coefficients', d_vector_final)
    fs_xml.release()


def write_normal_xml(file_xml_path, height, width, k_matrix_input, d_vector_input):
    folder_path = os.path.dirname(file_xml_path)
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)

    fs_xml = cv.FileStorage(file_xml_path, cv.FILE_STORAGE_WRITE)
    fs_xml.write('image_width', width)
    fs_xml.write('image_height', height)

    fs_xml.write('camera_matrix', k_matrix_input)
    d_vector_final = np.squeeze(np.asarray(d_vector_input))
    fs_xml.write('distortion_coefficients', d_vector_final)
    fs_xml.release()


if __name__ == '__main__':
    if len(sys.argv) < 6:  # 6 parameters
        print "not enough parameters"
        exit(-1)

    file_input = sys.argv[1]  # '/home/abacus/work/calibrate/fisheye_parameters.yaml'
    folder_path_source = sys.argv[2]  # '/home/abacus/work/calibrate/kalibr_camera/cam0/'
    output_folder = sys.argv[3]  # '/home/abacus/work/calibrate/undistorted/cam0/'
    file_xml_original_path = sys.argv[4]  # /home/abacus/work/calibrate/output_folder/ov580_original_camera_params.xml
    file_xml_normal_path = sys.argv[5]  # /home/abacus/work/calibrate/output_folder/ov580_norm_camera_params.xml

    fs = cv.FileStorage(file_input, cv.FILE_STORAGE_READ)
    fn_k_matrix = fs.getNode('K matrix')
    fn_d_vector = fs.getNode('D vector')
    k_matrix = fn_k_matrix.mat()
    d_vector = fn_d_vector.mat()
    fs.release()

    #
    list_files = get_all_file_path(folder_path_source)
    bGot = False
    row = 0
    column = 0
    for file_path in list_files:
        if not bGot:
            bGot = True
            img = cv.imread(folder_path_source + file_path)
            row, column, _ = img.shape
        undistorted(folder_path_source, file_path, output_folder, k_matrix, d_vector)

    # output xml files
    write_original_xml(file_xml_original_path, row, column, k_matrix, d_vector)
    write_normal_xml(file_xml_normal_path, row, column, k_matrix, d_vector)
