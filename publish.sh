#!/bin/bash
# Replace `your-topic-id` with the Pub/Sub topic you have created.
input="data/sample-laser-radar-measurement-data-1.txt"
while IFS= read -r line
do
  gcloud pubsub topics publish your-topic-id --message="$line" --ordering-key="kf"
done < "$input"