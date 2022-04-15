import cv2
import matplotlib.pyplot as plt
import PIL
from PIL import Image
import numpy as np
from scipy.io import savemat,loadmat
from tqdm import tqdm
import time
import math
# region growing for binarized data
# the coding of label start from 1
