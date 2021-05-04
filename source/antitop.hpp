#include "attack.hpp"
#include "target.hpp"

template<int length>
static double mean(RoundQueue<double, length> &vec) {
    double sum = 0;
    for (int i = 0; i < vec.size(); i++) {
        sum += vec[i];
    }
    return sum / length;
}

static double getFrontTime(const vector<double> time_seq, const vector<float> angle_seq) {
    double A = 0, B = 0, C = 0, D = 0;
    int len = time_seq.size();
    for (int i = 0; i < len; i++) {
        A += angle_seq[i] * angle_seq[i];
        B += angle_seq[i];
        C += angle_seq[i] * time_seq[i];
        D += time_seq[i];
        cout << "(" << angle_seq[i] << ", " << time_seq[i] << ") ";
    }
    double b = (A * D - B * C) / (len * A - B * B);
    cout << b << endl;
    return b;
}

void AttackBase::antiTop(float &delay_time = 0) {
    // 判断是否发生装甲目标切换。
    // 记录切换前一段时间目标装甲的角度和时间
    // 通过线性拟合计算出角度为0时对应的时间点
    // 通过两次装甲角度front_time为零的时间差计算陀螺旋转周期
    // 根据旋转周期计算下一次装甲出现在角度为零的时间点
    if last_
    float width = 0.0;
    if (target_box.type == TARGET_SMALL)
        width = 135;
    else
        width = 230;

    if (cv::norm(last_box.pixelCenterPt2f - target_box.pixelCenterPt2f) > width * 1.5) {
        auto front_time = getFrontTime(time_seq, angle_seq);
        auto once_periodms = front_time - last_front_time;
        top_periodms.push(once_periodms);
        auto periodms = mean(top_periodms);

        uint16_t shoot_delay = front_time + periodms * 2 - double(m_currentTimeStamp/1000);//单位可能要改
        if (abs(once_periodms - top_periodms[-1]) <= 50) {
            delay_time =  shoot_delay
        }
        time_seq.clear();
        angle_seq.clear();
        last_front_time = front_time;
    } 
    else {
        time_seq.emplace_back(m_currentTimeStamp);
        angle_seq.emplace_back(target_box.rYaw);
    }
    anti_top_cnt ++;
}
