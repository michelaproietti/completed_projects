from matplotlib import pyplot as plt
from skimage.transform import resize
import tensorflow as tf
import numpy as np
import pickle


def show_coils(kspaceslice, num_coils, cmap=None):
    data = tf.math.log(tf.math.abs(kspaceslice) + 1e-9)
    fig = plt.figure(figsize=(12,5))
    fig.suptitle('FIRST 4 COILS OF THE CONSIDERED SLICE', fontsize=14)
    for i, num in enumerate(num_coils):
        plt.subplot(1, len(num_coils), i + 1)
        plt.imshow(data[num], cmap=cmap)
        plt.axis('off')


@tf.function
def reconstruct_from_kspace(kspace_data, complex_flag=False, batched=False):
    if complex_flag:
        if batched:
            k_real, k_complex = tf.split(kspace_data, 2, axis=1)
        else:
            k_real, k_complex = tf.split(kspace_data, 2, axis=0) #splitta parte reale e parte complessa
        kspace_data = tf.complex(k_real, k_complex)
        
    kspace_data = tf.signal.ifftshift(kspace_data, axes=[-2, -1]) #swaps half-spaces for all axes listed
    ift = tf.signal.ifft2d(kspace_data)
    ift = tf.signal.fftshift(ift, axes=[-2, -1])
    reconstructed_magnitude_image = tf.math.abs(ift)
    
    return reconstructed_magnitude_image


@tf.function
def reconstruct_magnitude_image(kspace_slice, complex_flag=False, batched=False):
    if batched:
        kspace_slice = tf.transpose(kspace_slice, [0, 3, 1, 2])
    else:
        kspace_slice = tf.transpose(kspace_slice, [2, 0, 1])
        
    rec_slice = reconstruct_from_kspace(kspace_slice, complex_flag, batched)
    
    if batched:
        return tf.math.sqrt(tf.reduce_sum(tf.math.square(rec_slice), axis=1))
    else:
        return tf.math.sqrt(tf.reduce_sum(tf.math.square(rec_slice), axis=0))


# Min-max normalization
def min_max_norm(img):
    img_min = tf.expand_dims(tf.reduce_min(img, axis=[1,2]), axis=1)
    img_max = tf.expand_dims(tf.reduce_max(img,axis=[1,2]), axis=1)

    img_min = tf.expand_dims(img_min,axis=1)
    img_max = tf.expand_dims(img_max,axis=1)
    
    denominator = tf.subtract(img_max, img_min)
    numerator = tf.subtract(img, img_min)          # broadcast
    
    normalized_image = tf.divide(numerator, denominator)
    
    return normalized_image


def reconstruct_normalized_image(image):
    img = tf.expand_dims(reconstruct_magnitude_image(image, True, True), axis=-1)
    
    # Setting central pixel of the image to the mean of the surrounding points
    w = int(np.round(img.shape[1]/2))
    h = int(np.round(img.shape[2]/2))
    
    img = np.array(img)
    img[:,w,h] = 0.25 * (img[:,w-1,h] + img[:,w+1,h] + img[:,w,h-1] + img[:,w,h+1])
    img = tf.Variable(img)
    
    normalized_image = min_max_norm(img)
    
    return normalized_image


def SNR(image):
    signal_mean = tf.math.reduce_mean(image, axis=0)
    signal_std = tf.math.reduce_std(image, axis=0)
    return tf.math.reduce_mean(tf.where(signal_std == 0, 0, signal_mean/signal_std))
