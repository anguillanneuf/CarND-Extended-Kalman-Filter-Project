#include "FusionEKF.h"
#include "google/cloud/pubsub/subscriber.h"
#include "tools.h"

namespace pubsub = google::cloud::pubsub;

int main(int argc, char* argv[]) try {

  if (argc != 3) {
    std::cerr << "Usage: " << argv[0]
              << " <project-id> <subscription-id>\n";
    return 1;
  }

  std::string const project_id = argv[1];
  std::string const subscription_id = argv[2];

  pubsub::Subscriber subscriber = pubsub::Subscriber(
      pubsub::MakeSubscriberConnection(
          pubsub::Subscription(project_id, subscription_id)));

  using Eigen::VectorXd;
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;
  std::mutex mu;
  std::condition_variable cv;
  int message_count = 0;
  FusionEKF fusionEKF;
  Tools tools;

  auto session = subscriber.Subscribe(
      [&](pubsub::Message const& m, pubsub::AckHandler h) {

        std::cout << "\nRECEIVED: " << m.data();

        // Preprocess.
        Struct packages;
        packages = tools.PreprocessPackages(m.data());

        // Predict and update.
        fusionEKF.ProcessMeasurement(packages.meas);
        std::cout << "PREDICTION: " << fusionEKF.ekf_.x_.transpose();

        // Evaluate.
        rmse = tools.CalculateRMSEContinuous(fusionEKF.ekf_.x_,
                                             packages.gt.gt_values_,
                                             rmse,
                                             message_count);
        std::cout << "\nACCURACY - RMSE: " << rmse.transpose();

        std::unique_lock<std::mutex> lk(mu);
        ++message_count;
        lk.unlock();
        cv.notify_one();
        // Ack the message.
        std::move(h).ack();
        std::cout << "\n# ACK'ED: " << message_count << "\n";
      });

  std::unique_lock<std::mutex> lk(mu);
  cv.wait_for(lk,
              std::chrono::seconds (30),
              [&message_count] { return message_count > 10000; });

  lk.unlock();
  // Cancel the subscription session.
  session.cancel();
  // Wait for the session to complete, no more callbacks after this point.
  auto status = session.get();
  // Report any final status, blocking.
  std::cout << "Message count: " << message_count << ", status: " << status
            << "\n";

} catch (std::exception const& ex) {
  std::cerr << "Standard exception raised: " << ex.what() << "\n";
  return 1;
}
