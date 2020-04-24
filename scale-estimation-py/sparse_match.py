import cv2
import numpy as np
import math
import os
import pandas as pd

class SparseScaleEstimator:
    def __init__(self, cam_mat, ROI, actual_height):
        self.K = cam_mat
        self.actual_height = actual_height
        self.ROI = ROI
        self.fd = cv2.AKAZE_create(threshold = 0.0001)
        # self.fd = cv2.ORB_create(2000)

    def get_matches(self, img1, img2):
        kp1 = self.fd.detect(img1, None)
        kp1, desc1 = self.fd.compute(img1, kp1)

        kp2 = self.fd.detect(img2, None)
        kp2, desc2 = self.fd.compute(img2, kp2)

        bf = cv2.BFMatcher(cv2.NORM_HAMMING)
        matches = bf.knnMatch(desc1, desc2, k=2)
        #matches = bf.match(desc1, desc2)
        good = []
        for m, n in matches:
            if m.distance < 0.8 * n.distance:
    #         if m.distance < 0.8 * n.distance:
                good.append(m)
    #     for m in matches:
    #         good.append(m)
            
        return kp1, desc1, kp2, desc2, good

    def getRT(self, kp1, kp2, matches):
        srcPt = []
        dstPt = []
        for p in matches:
            srcPt.append(kp1[p.queryIdx].pt)
            dstPt.append(kp2[p.trainIdx].pt)

        srcPt = np.array(srcPt)
        dstPt = np.array(dstPt)
        E, mask = cv2.findEssentialMat(srcPt, dstPt, self.K)
        a = np.array([])
        ninliers, R, t, mask, tr = cv2.recoverPose(E, srcPt, dstPt, self.K, mask = mask, distanceThresh=50.)
        print("Get RT: {}".format(tr.shape))
        
        return ninliers, R, t, mask, srcPt, dstPt, tr

    #lets trianatgulate points to get 3D points
    def triangulate_pair(self, R,t, srcPtFiltered, dstPtFiltered):
        R0 = np.identity(3)
        t0 = np.zeros((3,1))
        ext0 = np.hstack((R0, t0))
        P0 = self.K.dot(ext0)
        ext1 = np.hstack((R, t))
        P1 = self.K.dot(ext1)
        
        X_points_homo = cv2.triangulatePoints(P0, P1, srcPtFiltered.transpose(), dstPtFiltered.transpose())
        X_points2 = X_points_homo/X_points_homo[3, :]
        
        return X_points2[:3, :]

    def filter_ground_ROI(self, pts2d):
        pos_indices = []
        for idx, pt in enumerate(pts2d):
            p = ( int(pt[0]), int(pt[1]) )
            if cv2.pointPolygonTest(self.ROI, p, 0) >= 0:
                pos_indices.append(idx)
                
        return pos_indices

    def get_H(self, src2d, dst2d):
        return cv2.findHomography(src2d, dst2d, method=cv2.RANSAC, ransacReprojThreshold=1.25)

    def calulate_nh(self, R, t, H):
        h = np.linalg.inv(self.K).dot(H).dot(self.K)
        
        B = np.zeros((9, 1), dtype=float)
        A = np.zeros((9, 4), dtype=float)
        
        B[0, 0] = R[0, 0]
        B[1, 0] = R[0, 1]
        B[2, 0] = R[0, 2]
        B[3, 0] = R[1, 0]
        B[4, 0] = R[1, 1]
        B[5, 0] = R[1, 2]
        B[6, 0] = R[2, 0]
        B[7, 0] = R[2, 1]
        B[8, 0] = R[2, 2]
        B = -B
        
        A[0, 0] = -1 * h[0, 0]
        A[0, 1] =  t[0, 0]
        A[0, 2] = 0
        A[0, 3] = 0
        
        A[1, 0] = -1 * h[0, 1]
        A[1, 1] = 0.
        A[1, 2] = t[0, 0]
        A[1, 3] = 0.
        
        A[2, 0] = -1 * h[0, 2]
        A[2, 1] = 0
        A[2, 2] = 0
        A[2, 3] = t[0, 0]
        
        A[3, 0] = -1 * h[1, 0]
        A[3, 1] = t[1, 0]
        A[3, 2] = 0
        A[3, 3] = 0
        
        A[4, 0] = -1 * h[1, 1]
        A[4, 1] = 0
        A[4, 2] = t[1, 0]
        A[4, 3] = 0
        
        A[5, 0] = -h[1, 2]
        A[5, 1] = 0
        A[5, 2] = 0
        A[5, 3] =t[1, 0]
        
        A[6, 0] = -1 * h[2, 0]
        A[6, 1] = t[2, 0]
        A[6, 2] = 0
        A[6, 3] = 0
        
        A[7, 0] = -1 * h[2, 1]
        A[7, 1] = 0
        A[7, 2] = t[2, 0]
        A[7, 3] = 0
        
        A[8, 0] = -1 * h[2, 2]
        A[8, 1] = 0
        A[8, 2] = 0
        A[8, 3] = t[2, 0]
        
        x, residuals, rank, s = np.linalg.lstsq(A, B, rcond=None)
        x = x[1:, :]
        h = 1. / np.linalg.norm(x)
        x_unit = x / np.linalg.norm(x)
        #print("residuals: {}".format(residuals))
        return x_unit, h, x

    def calculate_scale(self, src_img, target_img):
        kp1, desc1, kp2, desc2, matches = self.get_matches(src_img, target_img)
        ninliers, R, t, mask, mSrcPt, mDstPt = self.getRT(kp1, kp2, matches)

        mask2 = mask.flatten()
        mask2.dtype = bool
        # print("before: {}".format(mSrcPt.shape[0]) )
        mSrcPt = mSrcPt[mask2, :]
        mDstPt = mDstPt[mask2, :]
        # print("after: {}".format(mSrcPt.shape[0]) )

        X = self.triangulate_pair(R, t, mSrcPt, mDstPt)

        roi_indices = self.filter_ground_ROI(mSrcPt)

        roi_only_src = np.array([p.tolist() for idx, p in enumerate(mSrcPt) if idx in roi_indices])
        roi_only_dst = np.array([p.tolist() for idx, p in enumerate(mDstPt) if idx in roi_indices])

        if len(roi_only_src) >= 4:
            homo_roi_only, status_roi = self.get_H(roi_only_src, roi_only_dst)
        else:
            homo_roi_only = None

        if homo_roi_only is not None:
            n_roi, height_roi, n_not_unit = self.calulate_nh(R, t, homo_roi_only)
        
            #print(n_roi)
            #print(repr(n_not_unit) )
            # print(math.acos(n_roi.transpose().dot(ideal_norm)) * 180. / math.pi)
            #print("scale using roi: {:.3f}".format(self.actual_height/height_roi))


            return n_roi, n_not_unit, self.actual_height/height_roi, R, t

        else:
            return None


class FixedPitchEstimator(SparseScaleEstimator):

    def calculate_scale(self, src_img, target_img):
        angle = 3. * math.pi /180.

        T_actual_road = np.array([
            [1, 0., 0.],
            [0., math.cos(angle), -math.sin(angle)],
            [0., math.sin(angle), math.cos(angle)]
        ])

        T_road_actual = np.linalg.inv(T_actual_road)

        ideal_norm = np.array([ [0.], [1.], [0.] ])
        ideal_norm = T_road_actual.dot(ideal_norm)


        kp1, desc1, kp2, desc2, matches = self.get_matches(src_img, target_img)

        ninliers, R, t, mask, mSrcPt, mDstPt, tr = self.getRT(kp1, kp2, matches)

        mask2 = mask.flatten()      
        mask2.dtype = bool
        # print("before: {}".format(mSrcPt.shape[0]) )
        mSrcPt = mSrcPt[mask2, :]
        mDstPt = mDstPt[mask2, :]
        # print("after: {}".format(mSrcPt.shape[0]) )

        X = self.triangulate_pair(R, t, mSrcPt, mDstPt).transpose()
        roi_indices = self.filter_ground_ROI(mSrcPt)

        ground_X = X[roi_indices]
        n_points = ground_X.shape[0]

        print("there are {} points".format(n_points))

        ground_X = T_road_actual.dot(ground_X.transpose()).transpose()
        ys = ground_X[:, 1]

        best_score = -1000000000000000
        best_h = -1

        for idx1, y in enumerate(ys):
            score = 0
            for idx2, y2 in enumerate(ys):
                if idx1 != idx2:
                    diff = -50 * (y - y2)**2
                    score += math.exp(diff)

            if score > best_score:
                best_h = y
                best_score = score

        scale = 1.7/best_h
        n = ideal_norm * scale

        return n, R, t





class RansacScaleEstimator(SparseScaleEstimator):
    def calculate_scale(self, src_img, target_img, iterations=50, accepted_ratio=0.6):
        kp1, desc1, kp2, desc2, matches = self.get_matches(src_img, target_img)
        ninliers, R, t, mask, mSrcPt, mDstPt, tr = self.getRT(kp1, kp2, matches)

        mask2 = mask.flatten()
        mask2.dtype = bool
        # print("before: {}".format(mSrcPt.shape[0]) )
        mSrcPt = mSrcPt[mask2, :]
        mDstPt = mDstPt[mask2, :]
        # print("after: {}".format(mSrcPt.shape[0]) )

        X = self.triangulate_pair(R, t, mSrcPt, mDstPt).transpose()
        roi_indices = self.filter_ground_ROI(mSrcPt)

        #ground points
        ground_X = X[roi_indices]
        n_points = ground_X.shape[0]

        print("there are {} points".format(n_points))

        threshold = 0.2
        best_count = 0
        best_model = None 
        best_inliers = None
        ransac_count = 0

        #ok now run ransac
        b = np.ones((3,1), dtype=float)
        for i in range(iterations):
            model_idx = np.random.choice(np.arange(n_points), 3, replace=False)
            p = np.zeros((3, 3))
            p[0, :] = ground_X[model_idx[0], :] 
            p[1, :] = ground_X[model_idx[1], :]
            p[2, :] = ground_X[model_idx[2], :]

            norm = np.linalg.inv(p).dot(b)
            l = np.linalg.norm(norm)
            height = 1. / l

            score_idx = [i for i in range(n_points) if i not in model_idx]
            ys = ground_X[score_idx, :].dot(norm/l)
            #import pdb;pdb.set_trace()
            hdiff = np.abs(height - ys)
            # print(np.where(hdiff < threshold))
            inliers = np.where(hdiff < threshold)[0]#.shape[0]
            inliers_count =  inliers.shape[0]

            if inliers_count > best_count and inliers_count/(n_points-3.) > accepted_ratio:
                best_count = inliers_count
                best_model = height
                # best_inliers = inliers
                best_inliers = np.append(inliers, model_idx)

                ransac_count += 1

        
        best_P = ground_X[best_inliers, :]
        # print("best p {}".format(best_P))
        best_norm, _, _, _ = np.linalg.lstsq(best_P, np.ones( ( best_P.shape[0], 1)), rcond=None)
        # print("best norm: {}".format(best_norm))
        best_model = 1. / np.linalg.norm(best_norm)

        #return n_roi, n_not_unit, self.actual_height/height_roi, R, t
        return best_norm/np.linalg.norm(best_norm), best_norm, self.actual_height/ best_model, R, t, best_count, best_count*1./n_points, ransac_count       

class RansacScaleEstimator2(SparseScaleEstimator):
    def calculate_scale(self, src_img, target_img, iterations=50, accepted_ratio=0.65, ):
        #TODO: rem,over this
        np.random.seed(1)


        kp1, desc1, kp2, desc2, matches = self.get_matches(src_img, target_img)
        ninliers, R, t, mask, mSrcPt, mDstPt, tr = self.getRT(kp1, kp2, matches)

        mask2 = mask.flatten()
        mask2.dtype = bool
        # print("before: {}".format(mSrcPt.shape[0]) )
        mSrcPt = mSrcPt[mask2, :]
        mDstPt = mDstPt[mask2, :]
        # print("after: {}".format(mSrcPt.shape[0]) )

        X = self.triangulate_pair(R, t, mSrcPt, mDstPt).transpose()
        roi_indices = self.filter_ground_ROI(mSrcPt)

        # import pdb;pdb.set_trace()

        #ground points
        ground_X2 = X[roi_indices]
        tr  = tr / tr[3, :]
        tr = tr[:3, :].transpose()
        ground_X = tr[mask2, :][roi_indices, :] 
        n_points = ground_X.shape[0]

        print("there are {} points".format(n_points))

        # large_angles = []
        # small_angles = []
        # for i in range(n_points):
        #     p2 = t.flatten()
        #     p1 = np.zeros((3,))
        #     p3 = ground_X2[i, :]

        #     p31 = -p3
        #     p32 = p2 - p3
        #     # import pdb;pdb.set_trace()
        #     angle = math.acos(p31.dot(p32) / (np.linalg.norm(p31) * np.linalg.norm(p32) ) ) * 180 / math.pi
        #     # print("angle: {}".format(angle))

        #     if angle > 0.5:#0.7:
        #         large_angles.append(i)
        #     else:
        #         small_angles.append(i)

        # valid_ground_X = ground_X[large_angles, :]
        # invalid_ground_X = ground_X[small_angles, :]

        # n_points = valid_ground_X.shape[0]
        # ground_X = valid_ground_X.copy()

        # print("std valid: {} ".format(valid_ground_X[:, 1].std()))
        # print("std invalid: {} ".format(invalid_ground_X[:, 1].std()))

        # import pdb; pdb.set_trace()

        threshold = 0.2
        best_error = 10000000
        best_count = 0
        best_model = None 
        best_inliers = None

        ret_model = None
        ret_inliers = None
        ret_norm = None

        ransac_count = 0

        # print(ground_X[:, 1])

        print("there are {} points after filtering".format(n_points))

        #ok now run ransac
        b = np.ones((3,1), dtype=float)
        for i in range(iterations):
            model_idx = np.random.choice(np.arange(n_points), 3, replace=False)
           
            p = np.zeros((3, 3))
            p[0, :] = ground_X[model_idx[0], :] 
            p[1, :] = ground_X[model_idx[1], :]
            p[2, :] = ground_X[model_idx[2], :]

            norm = np.linalg.inv(p).dot(b)
            l = np.linalg.norm(norm)
            height = 1. / l

            score_idx = [i for i in range(n_points) if i not in model_idx]
            ys = ground_X[score_idx, :].dot(norm/l)
            #import pdb;pdb.set_trace()
            hdiff = np.abs(height - ys)
            # print(hdiff.mean())
            # print(np.where(hdiff < threshold))
            inliers = np.where(hdiff < threshold)[0]#.shape[0]
            inliers_count =  inliers.shape[0]

            if (inliers_count * 1. / (n_points-3)) > accepted_ratio:
                #refit with all points
                best_inliers = np.append(inliers, model_idx)
                best_P = ground_X[best_inliers, :]
                best_norm, _, _, _ = np.linalg.lstsq(best_P, np.ones( ( best_P.shape[0], 1)), rcond=None)

                best_l = np.linalg.norm(best_norm)
                best_height = 1. / best_l
                ys = ground_X[best_inliers, :].dot(best_norm/best_l)

                ransac_count += 1

                avg_hdiff = np.square(best_height - ys).mean()
                avg_hdiff = np.sqrt(avg_hdiff)
                if avg_hdiff < best_error:
                    best_error = avg_hdiff

                    ret_inliers = (inliers_count+3) * 1. / n_points
                    ret_model = best_height
                    ret_norm = best_norm

                    # print("Better err: {}".format(avg_hdiff))

        print("ransac count: {}".format(ransac_count))

        if ransac_count == 0:
            large_angles = []
            small_angles = []

            for i in range(n_points):
                p2 = t.flatten()
                p1 = np.zeros((3,))
                p3 = ground_X2[i, :]

                p31 = -p3
                p32 = p2 - p3
                # import pdb;pdb.set_trace()
                angle = math.acos(p31.dot(p32) / (np.linalg.norm(p31) * np.linalg.norm(p32) ) ) * 180 / math.pi
                # print("angle: {}".format(angle))

                if angle > 0.5:#0.7:
                    large_angles.append(i)
                else:
                    small_angles.append(i)

            valid_ground_X = ground_X[large_angles, :]
            invalid_ground_X = ground_X[small_angles, :]

            n_points = valid_ground_X.shape[0]
            ground_X = valid_ground_X.copy()

            print("reducing points to {}".format(n_points))

            b = np.ones((3,1), dtype=float)
            for i in range(iterations):
                model_idx = np.random.choice(np.arange(n_points), 3, replace=False)
               
                p = np.zeros((3, 3))
                p[0, :] = ground_X[model_idx[0], :] 
                p[1, :] = ground_X[model_idx[1], :]
                p[2, :] = ground_X[model_idx[2], :]

                norm = np.linalg.inv(p).dot(b)
                l = np.linalg.norm(norm)
                height = 1. / l

                score_idx = [i for i in range(n_points) if i not in model_idx]
                ys = ground_X[score_idx, :].dot(norm/l)
                #import pdb;pdb.set_trace()
                hdiff = np.abs(height - ys)
                # print(hdiff.mean())
                # print(np.where(hdiff < threshold))
                inliers = np.where(hdiff < threshold)[0]#.shape[0]
                inliers_count =  inliers.shape[0]

                if (inliers_count * 1. / (n_points-3)) > accepted_ratio:
                    #refit with all points
                    best_inliers = np.append(inliers, model_idx)
                    best_P = ground_X[best_inliers, :]
                    best_norm, _, _, _ = np.linalg.lstsq(best_P, np.ones( ( best_P.shape[0], 1)), rcond=None)

                    best_l = np.linalg.norm(best_norm)
                    best_height = 1. / best_l
                    ys = ground_X[best_inliers, :].dot(best_norm/best_l)

                    ransac_count += 1

                    avg_hdiff = np.square(best_height - ys).mean()
                    avg_hdiff = np.sqrt(avg_hdiff)
                    if avg_hdiff < best_error:
                        best_error = avg_hdiff

                        ret_inliers = (inliers_count+3) * 1. / n_points
                        ret_model = best_height
                        ret_norm = best_norm            



        return ret_norm / np.linalg.norm(ret_norm), ret_norm, self.actual_height/ret_model, R, t, ret_inliers, ret_inliers, ransac_count

        # best_P = ground_X[best_inliers, :]
        # # print("best p {}".format(best_P))
        # best_norm, _, _, _ = np.linalg.lstsq(best_P, np.ones( ( best_P.shape[0], 1)) )
        # # print("best norm: {}".format(best_norm))
        # best_model = 1. / np.linalg.norm(best_norm)

        # #return n_roi, n_not_unit, self.actual_height/height_roi, R, t
        # return best_norm/np.linalg.norm(best_norm), best_norm, self.actual_height/ best_model, R, t, best_count, best_count*1./n_points  


if __name__ == '__main__':
    #for dataset 00-02
    # fx = 718.856
    # fy = 718.856
    # cx = 607.1928
    # cy = 185.2157

    fx = 707.0912
    fy = 707.0912
    cx = 601.8873
    cy = 183.1104

    ideal_norm = np.array([ [0.], [1.], [0.] ])

    cam_mat = np.array([
            [fx, 0, cx],
            [0, fy, cy],
            [0,  0,  1]
        ])
    actual_height = 1.7

    seq_str = '06'
    img_data_dir = '/home/cy/Documents/data/kitti/data_odometry_gray/dataset/sequences/{}/image_0/'.format(seq_str)
    gt_file = '/home/cy/Documents/data/kitti/data_odometry_poses/{}.txt'.format(seq_str)

    nframes = len([f for f in os.listdir(img_data_dir) if f.endswith(".png")])

    seq = int(seq_str)

    import sys 
    sample_idx = int(sys.argv[1])
    fn = str(sample_idx).rjust(6, '0')
    fnm1 = str(sample_idx+1).rjust(6, '0')
    img1 = cv2.imread(os.path.join(img_data_dir,"{}.png".format(fn)), 0)
    img2 = cv2.imread(os.path.join(img_data_dir,"{}.png".format(fnm1)), 0)


    df_truth = pd.read_csv(gt_file, sep= ' ', header=None)[[3,7,11]]
    df_truth.rename(columns={3:'x', 7:'y', 11:'z'}, inplace=True)
    df_truth['translation'] = 1.
    for i in range(1, len(df_truth)):
        tx = math.pow(df_truth.iloc[i]['x'] - df_truth.iloc[i-1]['x'], 2) + math.pow(df_truth.iloc[i]['y'] - df_truth.iloc[i-1]['y'], 2) + math.pow(df_truth.iloc[i]['z'] - df_truth.iloc[i-1]['z'], 2) 
        df_truth.at[i, 'translation'] = math.sqrt(tx)

    ROI = np.array([
        [500, 230], [700, 230], [800+25, 380], [400-25, 380]
    ])
    ROI = ROI.reshape((-1, 1, 2))


    estimator4 = FixedPitchEstimator(cam_mat, ROI, 1.7)
    scale_fixed = estimator4.calculate_scale(img1, img2)

    print("fixed scale: {}".format(scale_fixed))

    #            return n_roi, n_not_unit, self.actual_height/height_roi, R, t
    # estimator = SparseScaleEstimator(cam_mat, ROI, 1.7)
    # unit_n, _, scale1, _, _ =  estimator.calculate_scale(img1, img2)    
    # print(unit_n    )
    # import pdb;pdb.set_trace()
    # print("scale 1 : {} angle 1: {}".format(scale1, math.acos(ideal_norm.transpose().dot(unit_n/np.linalg.norm(unit_n)) )* 180 / math.pi))     

    estimator2 = RansacScaleEstimator2(cam_mat, ROI, 1.7)
    all_scales = []
    for i in range(1):
        _, _, scale2, _, _,  inliers, ratio, ransac_count = estimator2.calculate_scale(img1, img2, 1000, accepted_ratio=0.6)
        # _, _, scale2, _, _,  inliers, ratio, ransac_count = estimator2.calculate_scale(img1, img2, 2000, accepted_ratio=0.6)
        all_scales.append(scale2)
        print("scale ransac : {} inlier_ratio: {}".format(scale2, ratio))

    print("mean scale: {}".format(np.array(all_scales).mean()))




    # estimator3 = RansacScaleEstimator(cam_mat, ROI, 1.7)
    # _, _, scale3, _, _,  inliers3, ratio3 = estimator3.calculate_scale(img1, img2, 50)


    # print("scale lousy ransac : {} inlier_ratio: {}".format(scale3, ratio3))


    # print("ground truth scale: {}".format(df_truth.at[sample_idx+1, 'translation']))
