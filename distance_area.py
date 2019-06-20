from __future__ import division
import numpy as np
import skimage
import matplotlib.pyplot as plt
import sys
import copy

def mask_image(image,graph=True):
    '''creates an image mask to compute area
    and distances'''
    #  fig,ax=skimage.filters.try_all_threshold(image)
    #  plt.show()
    thresh=skimage.filters.threshold_triangle(image)
    binary=image<thresh
    binary=binary.astype(int)
    if graph:
        plt.imshow(binary,cmap='gray')
    return binary

def area(image,hor_dim):
    '''computes surface area given by image
    hor_dim-the distance in desired units
    along the horizontal axis'''
    start=0
    end=image.shape[1]
    places=[]
    for col in range(image.shape[1]):
        ones=len(np.where(image[:,col]==1)[0])>0
        if ones>0:
            places.append(col)
    start=min(places)
    end=max(places)
    len_per_pixel=hor_dim/(end-start)
    return len_per_pixel*len_per_pixel*np.sum(image)

def distance(image,hor_dim,hor_center,ver_center,graph=True):
    '''computes moment arm distance given by image
    hor_dim-the distance in desired units
    along the horizontal axis'''
    places=[]
    for row in range(image.shape[0]):
        ones=len(np.where(image[row,:]==1)[0])>0
        if ones>0:
            places.append(row)
    start_ver=min(places)
    end_ver=max(places)
    places=[]
    for col in range(image.shape[1]):
        ones=len(np.where(image[:,col]==1)[0])>0
        if ones>0:
            places.append(col)
    start_hor=min(places)
    end_hor=max(places)
    len_per_pixel=hor_dim/(end_hor-start_hor)
    hor_adjustment=int(hor_center/len_per_pixel)
    ver_adjustment=int(ver_center/len_per_pixel)
    middle_ver=int((end_ver-start_ver)/2)+start_ver+ver_adjustment
    top_area=np.sum(image[:middle_ver,:])
    bottom_area=np.sum(image[middle_ver:,:])
    #top half
    top_half=0
    top_pixel=0
    for row in range(middle_ver):
        if top_half<(top_area/2):
            top_half+=np.sum(image[row,:])
            top_pixel=row
    #bottom half
    bottom_half=0
    bottom_pixel=0
    for row in range(middle_ver,image.shape[0]):
        if bottom_half<(bottom_area/2):
            bottom_half+=np.sum(image[row,:])
            bottom_pixel=row
    top_arm=(middle_ver-top_pixel)*len_per_pixel
    bottom_arm=(bottom_pixel-middle_ver)*len_per_pixel
    #  print top_pixel,bottom_pixel
    #  print top_arm,bottom_arm

    middle_hor=int((end_hor-start_hor)/2)+start_hor+hor_adjustment
    left_area=np.sum(image[:,:middle_hor])
    right_area=np.sum(image[:,middle_hor:])
    #left half
    left_half=0
    left_pixel=0
    for col in range(middle_hor):
        if left_half<(left_area/2):
            left_half+=np.sum(image[:,col])
            left_pixel=col
    #right half
    right_half=0
    right_pixel=0
    for col in range(middle_hor,image.shape[1]):
        if right_half<(right_area/2):
            right_half+=np.sum(image[:,col])
            right_pixel=col

    left_arm=(middle_hor-left_pixel)*len_per_pixel
    right_arm=(right_pixel-middle_hor)*len_per_pixel
    #  print left_pixel,right_pixel
    #  print left_arm,right_arm

    #graph
    if graph:
        plt.scatter(middle_hor,middle_ver,color='r',label='center of gravity')
        plt.scatter(middle_hor-hor_adjustment,middle_ver-ver_adjustment,color='b',label='center of geometry')
        plt.scatter(middle_hor,top_pixel,color='y')
        plt.scatter(middle_hor,bottom_pixel,color='y')
        plt.scatter(left_pixel,middle_ver,color='y')
        plt.scatter(right_pixel,middle_ver,color='y')
        plt.legend()
    print [len_per_pixel*len_per_pixel*x for x in [top_area,bottom_area,left_area,right_area]]
    return top_arm,bottom_arm,left_arm,right_arm
    

if __name__ == '__main__':
    pic=skimage.io.imread('z_dim.PNG')
    pic=skimage.color.rgb2gray(pic)
    mask=mask_image(pic)
    square_area=area(mask,0.781558)
    #  print square_area
    #  top_arm,bottom_arm,left_arm,right_arm=distance(mask,0.781558,0.00292,-0.02146)
    top_arm,bottom_arm,left_arm,right_arm=distance(mask,0.781558,0.00491,-0.00292)
    print top_arm,bottom_arm,left_arm,right_arm
    plt.show()
