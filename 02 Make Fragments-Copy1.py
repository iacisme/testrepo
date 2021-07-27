#!/usr/bin/env python
# coding: utf-8

# # <font style="color:rgb(64, 54, 245)"> **Make fragments** </font>
# 
# This example is based on the **Make Fragments** examples found on the Open3D website: http://www.open3d.org/docs/release/tutorial/reconstruction_system/make_fragments.html
# 
# The first step of the scene reconstruction system is to create fragments from short RGBD sequences.

# In[1]:


import os

import open3d as o3d


# # <font style="color:rgb(64, 54, 245)"> **Part 1 - Combine two images to make a PCD fragment** </font>
# 
# This example is going to combine two images, to prove the concept.
# 
# The next example will do the same thing a "for" loop.

# ## <font style="color:rgb(64, 54, 245)"> **Manually Registering Images** </font>

# ### <font style="color:rgb(64, 54, 245)"> **Read image pairs** </font>

# In[2]:


image_folder_path = '../../testing_fragements/images/'

depth_folder_path = '../../testing_fragements/depth/'


# In[3]:


# listdir() returns a list containing the names of the entries in the directory given by 'path'. 
# The list is in arbitrary order.It does not include the special entries '.' and '..' 
# even if they are present in the directory

# Returns the list of files in the image_folder_path
rgb_image_files = os.listdir(image_folder_path)

# Returns the list of files in the depth_folder_path
depth_files = os.listdir(depth_folder_path)


# In[4]:


if len(rgb_image_files) != len(depth_files):
    print("Warning, mis-matched imgage quantities")
    print("Total image files = ", rgb_image_files)
    print("Total depth files = ", depth_files)
    print("Numbers should be the same")
else:
    print("Total files to process = ", len(rgb_image_files) -1)


# In[5]:


# Sort both lists in assending order

rgb_image_files.sort()

depth_files.sort()


# ### <font style="color:rgb(64, 54, 245)"> **Register RGBD image pairs** </font>

# In[6]:


# What index number to start at.
# Remeber that we have 151 images, so the below number is the nth # index from 0 to 151.

rgb_image = 4

depth_file = 4

# Load a color image from the rgb_image_files

raw_color_image = o3d.io.read_image(image_folder_path + rgb_image_files[rgb_image])# Load a raw_depth_image from the raw_depth_image_files

raw_depth_image = o3d.io.read_image(depth_folder_path + depth_files[depth_file])
# In[7]:


# Testing the capabilities of "try / except" in program trouble-shooting

try:
    raw_color_image = o3d.io.read_image(image_folder_path + rgb_image_files[rgb_image])
    raw_depth_image = o3d.io.read_image(depth_folder_path + depth_files[depth_file])
    print("Images succesfully loaded")

except:
    print("Error opening color or depth image")    


# In[8]:


# Creaet an RGBD_Image object by combining both the image and depth

try:
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(raw_color_image, raw_depth_image, convert_rgb_to_intensity = False)
    print("RBGD Image created")

except:
    print("Error creating RGB Image")


# In[9]:


# Plot the colours using matplot lib
import matplotlib.pyplot as plt

plt.figure(figsize = [20,12])

# Prepare the plot of the colour image (converted to greyscale)
plt.subplot(1, 2, 1)
plt.title('Color Image')
plt.imshow(rgbd_image.color)

# Prepare the plot of the deoth image (converted to greyscale)
plt.subplot(1, 2, 2)
plt.title('Depth image')
plt.imshow(rgbd_image.depth)

plt.show()


# ### <font style="color:rgb(64, 54, 245)"> **Create and Display PCD image** </font>

# In[10]:


# Create a pointcloud object from the rgbd_image object, using the perspective and geometry from the pinhole camera method
try:
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image,
                                                         o3d.camera.PinholeCameraIntrinsic(
                                                             o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault)
                                                        )
    # The purpose of this script is to confirm that the point cloud file
    # actual has data saved in it. The pcd object is created regardless of the 
    # outcome
    if pcd is None:
        print("Error, no point cloud created")
    
    else:
        print("Point cloud file created")

except:
    print("Error creating point cloud")


# In[11]:


# Flip it, otherwise the pointcloud will be upside down

pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])


# In[12]:


#Draw the pointcloud

o3d.visualization.draw_geometries([pcd])


# ### <font style="color:rgb(64, 54, 245)"> **Write PCD to disk** </font>

# In[ ]:


#Start the pcd counter

d = 300 # In this example, the first picture in this sequence starts at 250.


# In[ ]:


# This save path will save a file with a conuter on it, %d.

pcd_save_path = '../../make_fragement_images/pcd_fragments/pcd_%d.pcd'%d


# In[ ]:


# Write the PCV save file

ret = o3d.io.write_point_cloud(pcd_save_path, pcd)

if ret == True:
    print("Point cloud saved")


# In[ ]:


# Increase the counter by 1 for the next save file

d += 1

d


# ## <font style="color:rgb(64, 54, 245)"> **Using a For Loop to register multiple PCDs** </font>

# In[ ]:


import os

import open3d as o3d


# In[ ]:


image_folder_path = '../../testing_fragements/images/'

depth_folder_path = '../../testing_fragements/depth/'

pcd_save_path = '../../testing_fragements/pcd_fragments/pcd_%d.pcd'%d


# In[ ]:


# listdir() returns a list containing the names of the entries in the directory given by 'path'. 
# The list is in arbitrary order.It does not include the special entries '.' and '..' 
# even if they are present in the directory

# Returns the list of files in the image_folder_path
rgb_image_files = os.listdir(image_folder_path)

# Returns the list of files in the depth_folder_path
depth_files = os.listdir(depth_folder_path)


# In[ ]:


# Sort both lists in assending order

rgb_image_files.sort()

depth_files.sort()


# In[ ]:


if len(rgb_image_files) != len(depth_files):
    print("Warning, mis-matched imgage quantities")
    print("Total image files = ", rgb_image_files)
    print("Total depth files = ", depth_files)
    print("Numbers should be the same")
else:
    print("Total files to process = ", len(rgb_image_files) -1) # The -1 prevents OBOB


# In[ ]:


total_pictures_to_process = len(depth_files) - 1

total_pictures_to_process


# In[ ]:


def read_in_an_image(picture, picture_path):
    print("The picture number is: ", picture)
    print("The picture path is: ", picture_path)
    


# In[ ]:


total_pictures_to_process = 3


# In[ ]:


for picture in range(total_pictures_to_process):
    
    print('\n')
    print(f"Processing point cloud number: ", picture)
    
    read_in_an_image(picture, image_folder_path)
    #raw_color_image = read_an_image(picture)
    #raw_depth_image - read_an_image(picture)
    


# In[ ]:





# In[ ]:


# Counter initializer. Make sure it matches the first sequence number

d =  356 #The first image in this sequence starts at # 356


# In[ ]:


# In the for loops below you must first define the 'len' of rgb_image_files (how many),
# Same for the 'depth-files.
# If not the for loops dont work (without using 'range')

# Initializers for counters
rgb_image = 0
depth_file = 0

for rgb_image in enumerate(rgb_image_files):
    for depth_file in enumerate(depth_files):

        print("\n")
        print("Creating RGB-D Image number: %d"%d)
        print("RGB_Image: ", rgb_image)
        print("depth_file: ", depth_file)       

        # Testing the capabilities of "try / except" in program trouble-shooting
        try:
            image = image_folder_path + rgb_image_files[rgb_image]
            depth = depth_folder_path + depth_files[depth_file]
            
            raw_color_image = o3d.io.read_image(image)
            raw_depth_image = o3d.io.read_image(depth)
            print("Images succesfully loaded")

        except:
            print("Error opening color or depth image") 

        # Creaet an RGBD_Image object by combining both the image and depth
        try:
            rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(raw_color_image, raw_depth_image)
            print("RBGD Image created")

        except:
            print("Error creating RGB Image")

        # Create a pointcloud object from the rgbd_image object, using the perspective and geometry from the pinhole camera method
        try:
            pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image,
                                                                 o3d.camera.PinholeCameraIntrinsic(
                                                                     o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault)
                                                                )
            # The purpose of this script is to confirm that the point cloud file
            # actual has data saved in it. The pcd object is created regardless of the 
            # outcome
            if pcd is None:
                print("Error, no point cloud created")

            else:
                print("Point cloud file created")

        except:
            print("Error creating point cloud")


        # Write the PCV save file
        ret = o3d.io.write_point_cloud(pcd_save_path, pcd)

        if ret == True:
            user_message = f"pcd_{d} saved to file"
            print(user_message)

    # Inrement the counter that will be used to number the saved PCDs
    d += 1


# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:


# This code comes from this reply on stackoverflow: 
# https://stackoverflow.com/questions/62809091/update-camera-intrinsic-parameter-in-open3d-python
# to define both the intrinsic and the extrinsic matrices

intrinsic = o3d.camera.PinholeCameraIntrinsic(w, h, fx, fy, cx, cy)

intrinsic.intrinsic_matrix = [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]

cam = o3d.camera.PinholeCameraParameters()

cam.intrinsic = intrinsic

cam.extrinsic = np.array([[0., 0., 0., 0.], [0., 0., 0., 0.], [0., 0., 0., 0.], [0., 0., 0., 1.]]) #Unless you have extrinsic values

pcd = o3d.geometry.create_point_cloud_from_rgbd_image(rgbd_image, 
                                                      cam.intrinsic, 
                                                      cam.extrinsic
                                                     )


# In[ ]:


# https://stackoverflow.com/questions/62912397/open3d-visualizing-multiple-point-clouds-as-a-video-animation
# This code is part of the non-blocking examples in open3d
# http://www.open3d.org/docs/0.8.0/tutorial/Advanced/non_blocking_visualization.html

vis = o3d.visualization.Visualizer()
vis.create_window()

# geometry is the point cloud used in your animaiton
geometry = o3d.geometry.PointCloud()
vis.add_geometry(geometry)

for i in range(icp_iteration):
    # now modify the points of your geometry
    # you can use whatever method suits you best, this is just an example
    geometry.points = pcd_list[i].points
    vis.update_geometry(geometry)
    vis.poll_events()
    vis.update_renderer()


# In[ ]:




