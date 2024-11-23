import cv2
import numpy as np
from matplotlib import pyplot as plt
import matplotlib.image as mpimg
 

def opencv_implementation_canny_edge_detection(image_path):
    # Read the image
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    
    # Check if image is loaded fine
    if img is None:
        print('Error opening image!')
        return
    
    # Apply GaussianBlur to reduce image noise and detail
    blurred_img = cv2.GaussianBlur(img, (5, 5), 1.4)
    
    # Apply Canny edge detection
    edges = cv2.Canny(blurred_img, 100, 200)
    
    # Display the original image and the edge-detected image
    plt.subplot(121), plt.imshow(img, cmap='gray')
    plt.title('Original Image'), plt.xticks([]), plt.yticks([])
    plt.subplot(122), plt.imshow(edges, cmap='gray')
    plt.title('Edge Image'), plt.xticks([]), plt.yticks([])
    
    plt.show()
    
    
# convert image to grayscale
def rgb2gray(rgb):
    r, g, b = rgb[:,:,0], rgb[:,:,1], rgb[:,:,2]
    gray = 0.2989 * r + 0.5870 * g + 0.1140 * b
    return gray

    
# create gaussian blur function
def gaussian_blur(img, kernel_size):
    # create a kernel
    kernel = np.ones((kernel_size, kernel_size), np.float32) / (kernel_size * kernel_size)
   
    img_blur = np.zeros_like(img)
    for i in range(kernel_size//2, img.shape[0] - kernel_size//2):
        for j in range(kernel_size//2, img.shape[1] - kernel_size//2):
            img_blur[i, j] = np.sum(img[i - kernel_size//2:i + kernel_size//2 + 1, j - kernel_size//2:j + kernel_size//2 + 1] * kernel)
            
    return img_blur

# create canny edge detection function
def canny_edge_detection(img):
    # apply gaussian blur
    img = gaussian_blur(img, 5)
    
    # find the gradient of the image
    sobel_x = np.array([[-1, 0, 1], [-2, 0, 2], [-1, 0, 1]])
    sobel_y = np.array([[1, 2, 1], [0, 0, 0], [-1, -2, -1]])
    
    G_x = np.zeros_like(img)
    G_y = np.zeros_like(img)
    
    for i in range(1, img.shape[0] - 1):
        for j in range(1, img.shape[1] - 1):
            G_x[i, j] = np.sum(img[i - 1:i + 2, j - 1:j + 2] * sobel_x)
            G_y[i, j] = np.sum(img[i - 1:i + 2, j - 1:j + 2] * sobel_y)
            
    # find the magnitude of the gradient
    G = np.sqrt(G_x ** 2 + G_y ** 2)
    
    # find the direction of the gradient
    theta = np.arctan2(G_y, G_x)
    
    # apply non-maximum suppression
    for i in range(1, img.shape[0] - 1):
        for j in range(1, img.shape[1] - 1):
            angle = theta[i, j] * 180 / np.pi
            if (0 <= angle < 22.5) or (157.5 <= angle <= 180):
                if (G[i, j] < G[i, j + 1]) or (G[i, j] < G[i, j - 1]):
                    G[i, j] = 0
            elif (22.5 <= angle < 67.5):
                if (G[i, j] < G[i - 1, j + 1]) or (G[i, j] < G[i + 1, j - 1]):
                    G[i, j] = 0
            elif (67.5 <= angle < 112.5):
                if (G[i, j] < G[i - 1, j]) or (G[i, j] < G[i + 1, j]):
                    G[i, j] = 0
            else:
                if (G[i, j] < G[i - 1, j - 1]) or (G[i, j] < G[i + 1, j + 1]):
                    G[i, j] = 0

   
    return G


# depth first search hyseteresis thresholding
def hysteresis_thresholding_dfs(pre_hyseteresis, low_threshold, high_threshold, print_stack=False):  
    

    img = pre_hyseteresis.copy()
    
    # apply hysteresis thresholding
    low_threshold = 0.01*np.max(img)
    high_threshold = 0.2 * np.max(img)
    
    visited_pixel = np.zeros_like(img)
    for i in range(1, img.shape[0] - 1):
        for j in range(1, img.shape[1] - 1):
            if img[i, j] >= high_threshold:
                img[i, j] = 255
                visited_pixel[i, j] = 1
                stack = [(i, j)]
                # Depth first search loop using stack and iterative approach instead of recursive
                while stack:
                    x, y = stack.pop()
                    # Pixel move loop
                    for m in range(-3, 3):
                        for n in range(-3, 3):
                            if  x+m < img.shape[0] and y+n < img.shape[1] and img[x + m, y + n] >= low_threshold and not visited_pixel[x + m, y + n]:
                                img[x + m, y + n] = 255
                                visited_pixel[x + m, y + n] = 1
                                stack.append((x + m, y + n)) 
                                if print_stack:
                                  print(stack)
                            
            
    return img

    

def main():
    # load image as np array
    org_img = mpimg.imread('cu.png')
    gray_img = rgb2gray(org_img)
    
    #Using value of 5 for kernel size, to reduce effect of blurr and reduce noise
    blurred_img = gaussian_blur(gray_img, 3)
    
    canny_img = canny_edge_detection(blurred_img)
    
    hysteresis_thresholding_img = hysteresis_thresholding_dfs(canny_img, 0.1 * np.max(canny_img), 0.3 * np.max(canny_img))
    
    
    #plt.imshow(canny_img, cmap=plt.get_cmap('gray'), vmin=0, vmax=1)
    
    # Display oringal image, blurred image and edge detected image
    plt.subplot(141), plt.imshow(org_img, cmap='gray')
    plt.title('Original Image'), plt.xticks([]), plt.yticks([])
    plt.subplot(142), plt.imshow(blurred_img, cmap='gray')
    plt.title('Blurred Image'), plt.xticks([]), plt.yticks([])
    plt.subplot(143), plt.imshow(canny_img, cmap='gray')
    plt.title('Edge Detected Image'), plt.xticks([]), plt.yticks([])
    plt.subplot(144), plt.imshow(hysteresis_thresholding_img, cmap='gray')
    plt.title('DFS Hystersis Applied'), plt.xticks([]), plt.yticks([])
    
    plt.show()  
     
     
    opencv_implementation_canny_edge_detection('cu.png')
    cv2.waitKey(0)
    
    
    
if __name__ == '__main__':
    main()    
    
    
    
 