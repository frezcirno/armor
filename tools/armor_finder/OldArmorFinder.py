import cv2
import math


class OldArmorFinder:
    @staticmethod
    def __shareEdge(t1, t2):
        if t1.pixelPts2f.tr == t2.pixelPts2f.tl and t1.pixelPts2f.br == t2.pixelPts2f.bl:
            return True

        if t2.pixelPts2f.tr == t1.pixelPts2f.tl and t2.pixelPts2f.br == t1.pixelPts2f.bl:
            return True

        return False

    def ArmorFinder(self):
        self.__colorMode = True   # 模式:红t蓝f
        self.__useDialte = False  # 是否膨胀

    def colorMode(self):
        return self.__colorMode

    def colorMode(self, val):
        self.__colorMode = val

    def useDialte(self):
        return self.__useDialte

    def useDialte(self, val):
        self.__useDialte = val

    @staticmethod
    def lightFeature(contour):
        res = {}
        res['src'] = contour

        # 像素大小
        cSize = cv2.contourArea(contour)
        rRect = cv2.minAreaRect(contour)
        rSize = rRect.size.area()
        rPts = rRect.points()  # 四个角点
        ctrPt = rRect.center  # 中心点

        res['size'] = cSize
        # 矩形度: 体现物体对其外接矩形的充满程度，反映一个物体与矩形相似程度的一个参数。
        res['rectangularity'] = cSize / rSize

        # 长边长度
        # 倾斜角: (-180, 0]
        # 外接矩形长宽比: >=1
        if rRect.size.width > rRect.size.height:
            # topPt = (rPts[2] + rPts[3]) / 2
            # btmPt = (rPts[0] + rPts[1]) / 2
            res['length'] = rRect.size.width
            res['angle'] = rRect.angle
            res['hwratio'] = rRect.size.width / rRect.size.height
        else:
            # topPt = (rPts[1] + rPts[2]) / 2
            # btmPt = (rPts[0] + rPts[3]) / 2
            res['length'] = rRect.size.height
            res['angle'] = rRect.angle - 90
            res['hwratio'] = rRect.size.height / rRect.size.width

    @staticmethod
    def isLight(feature):
        # 面积 >= 5
        if feature['size'] < 5:
            return False  # , 'size'

        # 最小外接矩形长宽比2/3～3/2
        if 0.6667 < feature['hwratio'] < 1.5:
            return False  # , 'hwratio'

        # 判断长度
        if feature['length'] < 3 or 800 < feature['length']:
            return False  # , 'length'

        # 判断倾斜角
        if feature['angle'] > -60 or feature['angle'] < -120:
            return False  # , 'angle'

        if feature['rectangularity'] < 0.5:
            return False  # , 'rectangularity'

        return True  # , ''

    @staticmethod
    def mkLight(feature):
        return feature['src']

    def findLights(self, frame):
        # 使用inRange对颜色进行筛选
        if self.__colorMode:
            '''红色'''
            bgrChecked = cv2.inRange(frame, (0, 0, 140), (70, 70, 255))
        else:
            '''蓝色'''
            bgrChecked = cv2.inRange(frame, (130, 100, 0), (255, 255, 65))

        if self.__useDialte:
            element = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
            bgrChecked = cv2.dilate(bgrChecked, element)

        contours = cv2.findContours(
            bgrChecked, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        features = map(self.lightFeature, contours)

        features = filter(self.isLight, features)

        return map(self.mkLight, features)

    @staticmethod
    def armorFeature(light1, light2):
        res = {}
        res['src'] = light1, light2

        diff = light2.centerPt - light1.centerPt
        avgAngle = (light1['angle'] + light2['angle']) / 2
        deltaAngle = abs(light1['angle'] - light2['angle'])

        res['avg_angle'] = avgAngle
        res['delta_angle'] = deltaAngle

        minLength = min(light1['length'], light2['length'])
        maxLength = max(light1['length'], light2['length'])
        avgLength = (light1['length'] + light2['length']) / 2
        deltaLength = abs(light1['length'] - light2['length'])

        res['min_length'] = minLength
        res['max_length'] = maxLength
        res['avg_length'] = avgLength
        res['delta_length'] = deltaLength

        orient = math.cos(math.radians(avgAngle)), \
            math.sin(math.radians(avgAngle))

        res['orientation'] = orient[0] * diff.x + orient[1] * diff.y

        res['v_angle'] = cv2.fastAtan2(
            abs(diff.y), abs(diff.x)) * 180 / math.pi
        res['length_ratio'] = maxLength / minLength

        return res

    @staticmethod
    def isArmor(feature):
        '''对两个灯条的错位度进行筛选'''
        if feature['delta_angle'] > 90:
            return False  # , 'delta_angle'

        if feature['orientation'] >= 25:
            return False  # , 'orientation'

        if feature['delta_angle'] > 23 and feature['min_length'] < 20:
            return False  # , 'delta_angle'

        if feature['delta_angle'] > 11 and feature['min_length'] >= 20:
            return False  # , 'delta_angle'

        if feature['length_ratio'] > 1.2:
            return False  # , 'length_ratio'

        if feature['v_angle'] > 25:
            return False  # , 'v_angle'

        # if AC2BC.x / minLength > 5:
        #     return False # , ''

        return True  # , ''

    @staticmethod
    def mkArmor(feature):
        return feature['src']

    def findArmor(self, lights):
        '''对灯条进行两两组合并筛选出预检测的装甲板'''
        lightCount = len(lights)
        features = [self.armorFeature(lights[i], lights[j])
                    for i in range(lightCount)
                    for j in range(i+1, lightCount)]
        features = filter(self.isArmor, features)
        return map(self.mkArmor, features)

    def detect(self, frame):
        lights = self.findLights(frame)
        sorted(lights, lambda light: light.centerPt.x)
        armors = self.findArmor(lights)
        return armors
