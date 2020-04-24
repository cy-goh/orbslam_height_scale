
import numpy as np
import cv2
from scipy.sparse import csr_matrix

class DenseMatcherScaleEstimator:
    def __init__(self, cam_mat, ROI):
        self.K =  cam_mat
        self.Kinv = np.linalg.inv(cam_mat)
        self.ROI = ROI

    def gx(self, R, T, n):
        G = self.K.dot(R + (T.dot(n.transpose()))).dot(self.Kinv)
        return G

    def get_valid_indices(self, R, t, n1, n2, n3, point_indices, img_dims):

        n = np.array([[n1], [n2], [n3]])    
        H = self.gx(R, t, n)
        
        uv = np.ones( (3, point_indices.shape[0]))
        uv[:2, :] = point_indices.transpose()
                    
        target_uv = H.dot(uv)
        target_uv /= target_uv[2, :]

        row_indices_valid = target_uv[1, :] < img_dims[0]
        
        valid_target = target_uv.transpose()[:, :2]#[row_indices_valid]
        
        return row_indices_valid, valid_target

    def eval_jacobian_I(self, grad_x, grad_y, transformed_indices):
        npoints = transformed_indices.shape[0]
        
        rows = []
        cols = []
        data = []
        
        transformed_indices2 = np.around(transformed_indices).astype(int)
        for row in range(npoints):
            x, y = transformed_indices2[row]

            rows.append(row)
            rows.append(row)
            cols.append(2*row)
            cols.append(2 * row + 1)
            data.append(grad_x[y, x])
            data.append(grad_y[y, x])        
        
        JI = csr_matrix((data, (rows, cols)) , shape = (npoints, npoints * 2),  dtype='float')
             
        return JI

    def eval_jacobian_I_fast(self, grad_x, grad_y, transformed_indices):
        npoints = transformed_indices.shape[0]
        transformed_indices2 = np.around(transformed_indices).astype(int)

        rows = np.repeat(np.arange(npoints), 2)
        cols = np.arange(npoints * 2)
        
        gx = grad_x[transformed_indices2[:, 1], transformed_indices2[:, 0]].reshape((-1, 1))
        gy = grad_y[transformed_indices2[:, 1], transformed_indices2[:, 0]].reshape((-1, 1))
        
        data = np.hstack((gx, gy)).flatten()
        JI = csr_matrix((data, (rows, cols)) , shape = (npoints, npoints * 2),  dtype='float')

        return JI


    def eval_jacobian_f(self, H, point_indices):
        h = H.flatten()
        
        Jf = np.zeros( (point_indices.shape[0] * 2, 9) )
        
        for row in range(point_indices.shape[0]):
            x = point_indices[row, 0]
            y = point_indices[row, 1]
            
            Jf[2 * row, 0] = x / (h[6] * x + h[7] * y + h[8])
            Jf[2 * row, 1] = y / (h[6] * x + h[7] * y + h[8])
            Jf[2 * row, 2] = 1. / (h[6] * x + h[7] * y + h[8] )
    #         Jf[2 * row, 3] = 0
    #         Jf[2 * row, 4] = 0
    #         Jf[2 * row, 5] = 0
            n = h[0] * x + h[1] * y + h[2]
            d = (h[6] * x + h[7] * y + h[8])**2
            Jf[2 * row, 6] = -(n / d) * x
            Jf[2 * row, 7] = -(n / d) * y
            Jf[2 * row, 8] = -(n / d)
            
    #         Jf[2 * row + 1, 0] = 0
    #         Jf[2 * row + 1, 1] = 0
    #         Jf[2 * row + 1, 2] = 0
            Jf[2 * row + 1, 3] = x / (h[6] * x + h[7] * y + h[8])
            Jf[2 * row + 1, 4] = y / (h[6] * x + h[7] * y + h[8])
            Jf[2 * row + 1, 5] = 1 / (h[6] * x + h[7] * y + h[8])
            n2 = h[3] * x + h[4] * y + h[5]
            Jf[2 * row + 1, 6] = -(n2 / d) * x
            Jf[2 * row + 1, 7] = -(n2 / d) * y
            Jf[2 * row + 1, 8] = -n2 /d
        #print("JF_slow")
        #print(Jf[:2])
        return Jf

    def eval_jacobian_f_fast(self, H, point_indices):
        h = H.flatten()
        
        Jf = np.zeros( (point_indices.shape[0] * 2, 9) )
        
        xy1 = np.hstack((point_indices, np.ones((point_indices.shape[0], 1))) )
        h678 = h[-3:]
        h012 = h[:3]
        h345 = h[3:6]
        
        #print(xy1)
        
        d1 = (xy1 * h678).sum(axis=1).reshape((-1, 1))  #(h[6] * x + h[7] * y + h[8])
        
        #print(xy1.shape)
        #print(d1.shape)
        Jf[::2, :3] = xy1 / d1
        
        n = (h012 * xy1).sum(axis=1).reshape((-1, 1)) 
        n2 = (h345 * xy1).sum(axis=1).reshape((-1, 1)) 
        d = d1**2
        
        Jf[::2, -3:] = -(n/d) * xy1
        
        Jf[1::2, 3:6] = xy1 / d1
        Jf[1::2, -3:] = -(n2/d) * xy1 
        return Jf

    def eval_jacobian_g(self, R, t, n1, n2, n3):
        f1 = self.K[0, 0]
        f2 = self.K[1, 1]
        c1 = self.K[0, 2]
        c2 = self.K[1, 2]
        
        fi1 = self.Kinv[0 ,0]
        fi2 = self.Kinv[1, 1]
        ci1  = self.Kinv[0, 2]
        ci2 =  self.Kinv[1, 2]
        
        J = np.zeros( (9, 3) )
        
        x = t[0, 0]
        y = t[1, 0]
        z = t[2, 0]
        
        J[0, 0] = f1 * fi1 * (x) + c1 * fi1 * (z)
        J[0, 1] = 0
        J[0, 2] = 0
        
        J[1, 0] = 0
        J[1, 1] = f1 * fi2 * (x) + c1 * fi2 * (z)
        J[1, 2] = 0
        
        J[2, 0] = f1 * (ci1 * x) + c1 * (ci1 * z) 
        J[2, 1] = f1 * (ci2 * x) + c1 * (ci2 * z) 
        J[2, 2] = f1 * (x) + c1 * (z)
        
        J[3, 0] = f2 * fi1 * (y) + c2 * fi1 * (z)
        J[3, 1] = 0
        J[3, 2] = 0
        
        J[4, 0] = 0
        J[4, 1] = f2 * fi2 * (y) + c2 * fi2 * (z)
        J[4, 2] = 0
        
        J[5, 0] = f2 * ci1 * (y) + c2 * ci1 * (z) 
        J[5, 1] = f2 * ci2 * (y) + c2 * ci2 * (z)
        J[5, 2] = f2 * (y) + c2 * (z)
        
        J[6, 0] = fi1 * (z)
        J[6, 1] = 0
        J[6, 2] = 0
        
        J[7, 0] = 0 #fi2 * (z)
        J[7, 1] = fi2 * z 
        J[7, 2] = 0 
        
        J[8, 0] = ci1 * (z)
        J[8, 1] = ci2 * (z)
        J[8, 2] = z
        
        return J
 
    def eval_jacobian(self, R, t, n1, n2, n3, point_indices, gradientx, gradienty, transformed_indices):
        n = np.array([[n1], [n2], [n3]])
        H = self.gx(R, t, n)

        Jf = self.eval_jacobian_f(H, point_indices)
        Jg = self.eval_jacobian_g(R, t, n1, n2, n3)
        JI = self.eval_jacobian_I(gradientx, gradienty, transformed_indices)
        J = JI.dot(Jf.dot(Jg))

        return J

    def eval_jacobian_fast(self, R, t, n1, n2, n3, point_indices, gradientx, gradienty, transformed_indices):
        n = np.array([[n1], [n2], [n3]])
        H = self.gx(R, t, n)
        
        Jf = self.eval_jacobian_f_fast(H, point_indices)
        Jg = self.eval_jacobian_g(R, t, n1, n2, n3)
        JI2 = self.eval_jacobian_I_fast(gradientx, gradienty, transformed_indices)
        J = JI2.dot(Jf.dot(Jg))

        return J

    def get_point_indices(self, img_dim):
        h, w = img_dim
        point_indices = []
        for r in range(h):
            for c in range(w):
                p = (c, r)
                if cv2.pointPolygonTest(self.ROI, p, 0) >= 0:
                    point_indices.append([c, r])
        
        point_indices = np.array(point_indices)
        point_indices = point_indices#[::2]
        return point_indices

    def get_image_gradient(self, img):
        gx = cv2.Scharr(img, cv2.CV_64F, 1, 0)
        gy = cv2.Scharr(img, cv2.CV_64F, 0, 1)
        
        #gx = cv2.Sobel(img,cv2.CV_64F,1,0,ksize=3)
        #gy = cv2.Sobel(img,cv2.CV_64F,0,1,ksize=3)
        return gx, gy

    def get_observations(self, img, point_indices):
        return img[point_indices[:,1], point_indices[:,0]]

    def huber_weight(self, error, k2 = (50./255.) **2):
        error = error * error

        w = np.ones(error.shape)

        indices = np.where(error >= k2)[0]
        w[indices] = np.sqrt(k2) / np.sqrt(error[indices])

        return w
    
    def optimize_scale(self, R, t, n1, n2, n3, lamb, max_iters, src_img, target_img, ref_height = 1.7, verbose=True):
        if verbose:
            print("current scale: {}".format(1.7 * np.linalg.norm(np.array([[n1], [n2], [n3]]))))

        point_indices = self.get_point_indices(src_img.shape)


        grad_x, grad_y = self.get_image_gradient(target_img)

        last_error = 1000000
        # last_delta = [0., 0., 0.]
    
        for r in range(max_iters):
            valids, transformed_indices = self.get_valid_indices(R, t, n1, n2, n3, point_indices, src_img.shape)
            cur_indices = point_indices[valids]
            
            expected_observations = self.get_observations(target_img, np.around(transformed_indices).astype(int)).astype('float')
            observations = self.get_observations(src_img, cur_indices).astype('float')

            err = observations- expected_observations
            huber_w = self.huber_weight(err)

            J = self.eval_jacobian_fast(R, t, n1, n2, n3, cur_indices, grad_x, grad_y, transformed_indices)
            J *= huber_w.reshape((-1,1))            

            JT = J.transpose()
            JJ = JT.dot(J)

            # print("shape of T: {}".format(J.shape))
                                           
            # delta = np.linalg.inv(JJ + lamb * np.diag(np.diag(JJ)) ).dot(JT.dot(observations - expected_observations) )
            delta = np.linalg.inv(JJ + lamb * np.diag(np.diag(JJ)) ).dot(JT.dot(huber_w * err) )

            n1 += delta[0]
            n2 += delta[1]
            n3 += delta[2]
                    
            err = observations - expected_observations
            err = (err * err).sum()
            
            if err <= last_error:
                lamb /= 3.
                lamb = max(0.00001, lamb)
            else:
                lamb *= 2.
                lamb = min(50, lamb)
                
            last_error = err
            
            if verbose:
                print("iter {} err: {} current scale: {} lambda: {} points: {}".format(
                    r, err, 1.7 * np.linalg.norm(np.array([[n1], [n2], [n3]])), lamb, cur_indices.shape[0]))

        normal = np.array([[n1], [n2], [n3]])
        scale = 1.7 * np.linalg.norm(normal)
        return normal, scale, np.sqrt(err/point_indices.shape[0] )