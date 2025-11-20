#include "orbbec_lidar/frame.hpp"

#include <memory>

#include "detail/frame.hpp"

namespace ob_lidar {
Frame::Frame(std::unique_ptr<detail::FrameImpl> impl)
    : impl_(std::move(impl)) {}

Frame::Frame() : impl_(nullptr) {}

const uint8_t* Frame::data() const { return impl_->data(); }
uint8_t* Frame::data() { return impl_->data(); }

size_t Frame::size() { return impl_->size(); }

bool Frame::isNull() { return false; }

Frame::~Frame() = default;
void Frame::copyMetaFrom(const std::shared_ptr<Frame>& other) {
    impl_->copyMetaFrom(other->impl_.get());
}
uint16_t Frame::frameId() const { return impl_->frameId(); }

uint8_t Frame::syncMode() const { return impl_->syncMode(); }

LidarFrameType Frame::type() { return impl_->type(); }

std::chrono::nanoseconds Frame::timestamp() { return impl_->timestamp(); }

NullFrame::NullFrame() = default;

NullFrame::~NullFrame() = default;

LidarFrameType NullFrame::type() { return LidarFrameType::UNKNOWN; }

const uint8_t* NullFrame::data() const { return empty_data_; }

uint8_t* NullFrame::data() { return empty_data_; }

size_t NullFrame::size() { return 0; }

bool NullFrame::isNull() { return true; }

void NullFrame::copyMetaFrom(const std::shared_ptr<Frame>& other) {
    (void)other;
}

std::chrono::nanoseconds NullFrame::timestamp() {
    return std::chrono::nanoseconds(0);
}

PointCloudFrame::PointCloudFrame(
    std::unique_ptr<detail::PointCloudFrameImpl> impl)
    : Frame(std::move(impl)) {}

PointCloudFrame::~PointCloudFrame() = default;

void PointCloudFrame::copyMetaFrom(const std::shared_ptr<Frame>& other) {
    if (const auto impl =
            dynamic_cast<detail::PointCloudFrameImpl*>(impl_.get())) {
        if (const auto other_impl = dynamic_cast<detail::PointCloudFrameImpl*>(
                other->impl_.get())) {
            impl->copyMetaFrom(other_impl);
        }
    }
}

double PointCloudFrame::getAngleScale() const {
    if (const auto impl =
            dynamic_cast<detail::PointCloudFrameImpl*>(impl_.get())) {
        return impl->getAngleScale();
    }
    return 0.0;
}

double PointCloudFrame::getDistanceScale() const {
    if (const auto impl =
            dynamic_cast<detail::PointCloudFrameImpl*>(impl_.get())) {
        return impl->getDistanceScale();
    }
    return 0.0;
}

LidarPointCloud PointCloudFrame::toPointCloud() const {
    if (const auto impl =
            dynamic_cast<detail::PointCloudFrameImpl*>(impl_.get())) {
        return impl->toPointCloud();
    }
    return {};
}

ScanFrame::ScanFrame(std::unique_ptr<detail::ScanFrameImpl> impl)
    : Frame(std::move(impl)) {}

ScanFrame::~ScanFrame() = default;

LidarScan ScanFrame::toScan() const {
    if (const auto impl = dynamic_cast<detail::ScanFrameImpl*>(impl_.get())) {
        return impl->toScan();
    }
    return {};
}

void ScanFrame::copyMetaFrom(const std::shared_ptr<Frame>& other) {
    if (const auto impl = dynamic_cast<detail::ScanFrameImpl*>(impl_.get())) {
        if (const auto other_impl =
                dynamic_cast<detail::ScanFrameImpl*>(other->impl_.get())) {
            impl->copyMetaFrom(other_impl);
        }
    }
}

void ScanFrame::filterScanSmoothly(std::vector<uint16_t>& distances,
                                   uint8_t window_size) {
    int count = distances.size();
    std::vector<int16_t> src_point_dist;

    src_point_dist.resize(count);
    for (int i = 0; i < count; i++) {
        if (i < count) {
            src_point_dist[i] = distances[i];
        }
    }
    for (int i = (window_size / 2); i < (count - (window_size / 2)); i++) {
        int half_window = window_size / 2;
        int range_sum = 0, range_square_sum = 0;
        int range_sum_ave = 0, range_square_ave = 0;
        for (int n = (i - half_window); n < i + half_window + 1; n++) {
            range_sum = range_sum + src_point_dist[n];
            range_square_sum =
                range_square_sum + src_point_dist[n] * src_point_dist[n];
            ;
        }
        range_sum_ave = range_sum / window_size;
        range_square_ave = range_square_sum / window_size;

        int32_t range_diff = 0;
        int32_t range_diff_sum = 0;
        for (int n = (i - half_window); n < i + half_window; n++) {
            range_diff = (src_point_dist[n + 1] >= src_point_dist[n])
                             ? (src_point_dist[n + 1] - src_point_dist[n])
                             : (src_point_dist[n] - src_point_dist[n + 1]);
            range_diff_sum = range_diff_sum + range_diff;
        }
        float xita = 0;

        if (0 != range_diff_sum)
            xita = (float)window_size * 1000 / range_diff_sum;

        float gf_a, gf_b, div_temp;
        div_temp = (range_square_ave - range_sum_ave * range_sum_ave + xita);
        if (0 != div_temp)
            gf_a = (float)(range_square_ave - range_sum_ave * range_sum_ave) /
                   (div_temp);

        gf_b = range_sum_ave - gf_a * range_sum_ave;

        int range_final = gf_a * src_point_dist[i] + gf_b;

        distances[i] = range_final;
    }
}
}  // namespace ob_lidar
