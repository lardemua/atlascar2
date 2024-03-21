import cv2
import numpy as np
import imutils
import tqdm
import os
from moviepy.editor import ImageSequenceClip


class VideoStitcher:
    def __init__(self, left_video_in_path, right_video_in_path, video_out_width=800, display=True):
        # Initialize arguments
        self.left_video_in_path = left_video_in_path
        self.right_video_in_path = right_video_in_path
        # self.video_out_path = video_out_path
        self.video_out_width = video_out_width
        self.display = display

        self.saved_homo_matrix = np.array([[ 6.19742530e-01, -6.64826906e-02,  7.57627449e+02],
 [-4.12016621e-03,  9.02840678e-01,  2.01152250e+00],
 [-3.47141283e-04, -1.36157913e-05,  1.00000000e+00]])

        # Initialize the saved homography matrix
        # self.saved_homo_matrix = np.array([[ 5.40304579e-01, -2.66912257e-01,  8.46703839e+02],
        #                                     [-9.64704842e-04,  8.91943912e-01,  1.34286931e+00],
        #                                     [-3.58766786e-04, -3.54147781e-05,  1.00000000e+00]])

    def stitch(self, images, ratio=0.75, reproj_thresh=4.0):
        # Unpack the images
        (image_b, image_a) = images

        # If the saved homography matrix is None, then we need to apply keypoint matching to construct it
        if self.saved_homo_matrix is None:
          
            # Detect keypoints and extract
            (keypoints_a, features_a) = self.detect_and_extract(image_a)
            (keypoints_b, features_b) = self.detect_and_extract(image_b)

            # Match features between the two images
            matched_keypoints = self.match_keypoints(keypoints_a, keypoints_b, features_a, features_b, ratio, reproj_thresh)

            # If the match is None, then there aren't enough matched keypoints to create a panorama
            if matched_keypoints is None:
                return None

            # Save the homography matrix
            self.saved_homo_matrix = matched_keypoints[1]
            # print(self.saved_homo_matrix)
        # Apply a perspective transform to stitch the images together using the saved homography matrix
        output_shape = (image_a.shape[1] + image_b.shape[1], image_a.shape[0])
        result = cv2.warpPerspective(image_a, self.saved_homo_matrix, output_shape)
        result[0:image_b.shape[0], 0:image_b.shape[1]] = image_b
        # self.saved_homo_matrix = None
        
        # Return the stitched image
        return result

    @staticmethod
    def detect_and_extract(image):
        # Detect and extract features from the image (DoG keypoint detector and SIFT feature extractor)
        descriptor = cv2.SIFT_create()
        (keypoints, features) = descriptor.detectAndCompute(image, None)

        # Convert the keypoints from KeyPoint objects to numpy arrays
        keypoints = np.float32([keypoint.pt for keypoint in keypoints])

        # Return a tuple of keypoints and features
        return (keypoints, features)

    @staticmethod
    def match_keypoints(keypoints_a, keypoints_b, features_a, features_b, ratio, reproj_thresh):
        # Compute the raw matches and initialize the list of actual matches
        matcher = cv2.DescriptorMatcher_create("BruteForce")
        raw_matches = matcher.knnMatch(features_a, features_b, k=2)
        matches = []

        for raw_match in raw_matches:
            # Ensure the distance is within a certain ratio of each other (i.e. Lowe's ratio test)
            if len(raw_match) == 2 and raw_match[0].distance < raw_match[1].distance * ratio:
                matches.append((raw_match[0].trainIdx, raw_match[0].queryIdx))

        # Computing a homography requires at least 4 matches
        if len(matches) > 4:
            # Construct the two sets of points
            points_a = np.float32([keypoints_a[i] for (_, i) in matches])
            points_b = np.float32([keypoints_b[i] for (i, _) in matches])

            # Compute the homography between the two sets of points
            (homography_matrix, status) = cv2.findHomography(points_a, points_b, cv2.RANSAC, reproj_thresh)

            # Return the matches, homography matrix and status of each matched point
            return (matches, homography_matrix, status)

        # No homography could be computed
        return None

    @staticmethod
    def draw_matches(image_a, image_b, keypoints_a, keypoints_b, matches, status):
        # Initialize the output visualization image
        (height_a, width_a) = image_a.shape[:2]
        (height_b, width_b) = image_b.shape[:2]
        visualisation = np.zeros((max(height_a, height_b), width_a + width_b, 3), dtype="uint8")
        visualisation[0:height_a, 0:width_a] = image_a
        visualisation[0:height_b, width_a:] = image_b

        for ((train_index, query_index), s) in zip(matches, status):
            # Only process the match if the keypoint was successfully matched
            if s == 1:
                # Draw the match
                point_a = (int(keypoints_a[query_index][0]), int(keypoints_a[query_index][1]))
                point_b = (int(keypoints_b[train_index][0]) + width_a, int(keypoints_b[train_index][1]))
                cv2.line(visualisation, point_a, point_b, (0, 255, 0), 1)

        return visualisation

    def run(self):

        left_video = self.left_video_in_path
        right_video = self.right_video_in_path
  
        print('[INFO]: Stitching ...')

   
        if not(left_video is None and right_video is None):
              
            stitched_frame = self.stitch([left_video, right_video])       
            stitched_frame = imutils.resize(stitched_frame, width=self.video_out_width)

            return stitched_frame
        
