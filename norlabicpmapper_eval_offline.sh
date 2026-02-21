#!/bin/bash

input_path_host=/media/mabox/SSD_shared/fomo-data


process_trajectory() {
    local trajectory=$1
    local output_path_host="/home/mabox/output/norlabicpmapper-${trajectory}"

    for date_dir in "${input_path_host}"/*/; do
        date=$(basename "${date_dir}")
        for dataset_dir in "${date_dir}${trajectory}_"*/; do
            [ -d "${dataset_dir}" ] || continue
            dataset=$(basename "${dataset_dir}")
            echo $dataset
            mkdir -p "${output_path_host}/${date}/${dataset}"
            echo "Processing dataset ${date}/${dataset}"

            log_file="${output_path_host}/${date}/${dataset}/output.log"
            echo "START_TIME: $(date +%s)" > "$log_file"

            docker run -it \
                    --name slam \
                    -it \
                    --rm \
                    --privileged \
                    --network host \
                    -e SSH_AUTH_SOCK=${SSH_AUTH_SOCK} \
                    -v "${dataset_dir}":/data \
                    -v "${output_path_host}/${date}/${dataset}":/output \
                    -v "${dataset_dir}/calib":/calib \
                    -e NAMESPACE="" \
                    -e IS_MAPPING=1 \
                    -e STORAGE_PATH=/output \
                    norlab/norlabicpmapper_offline /bin/bash -c "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 launch norlab_icp_mapper_ros offline_mapper.launch.py bag_path:=/data" \
                    2>&1 | tee "${output_path_host}/${date}/${dataset}/docker.log"
            echo "END_TIME: $(date +%s)" >> "$log_file"
        done
    done
}

process_trajectory "red"
process_trajectory "blue"
process_trajectory "green"
process_trajectory "magenta"


input_path_host=/home/mabox/data/
process_trajectory "yellow"


input_path_host=/media/mabox/SSD_Matej/fomo-lidar
process_trajectory "orange"