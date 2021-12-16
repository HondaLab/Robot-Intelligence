import cv2
import matplotlib.pyplot as plt

a = cv2.imread('robot1.jpg')
plt.imshow(a)
plt.savefig('robot1.eps',format='eps',dpi=1000)
