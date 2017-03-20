#! /usr/bin/env ruby
require 'orocos'
require 'orocos/log'
require 'vizkit'
require "transformer/runtime"

include Orocos
Orocos::CORBA.max_message_size = 80000000

if ARGV.size < 1 then 
    puts "usage: #{__FILE__} logfolder"
    exit
end

log = Orocos::Log::Replay.open(ARGV)
log.track(false)

Orocos.initialize

# find ports
imu_samples = log.imu_kvh_1750.calibrated_sensors
imu_samples.tracked = true

log.transformer_broadcaster.rename('transformer_broadcaster_log') if log.has_task? "transformer_broadcaster"
log.orientation_estimator_ikf.rename('orientation_estimator_log') if log.has_task? "orientation_estimator_ikf"

Orocos.run "pose_estimation::OrientationEstimator" => 'orientation_estimator' do

    Orocos.transformer.load_conf("#{ENV['AUTOPROJ_CURRENT_ROOT']}/slam/orogen/pose_estimation/scripts/config/transforms_live_static.rb")
    Orocos.conf.load_dir("#{ENV['AUTOPROJ_CURRENT_ROOT']}/slam/orogen/pose_estimation/scripts/config/")

    orientation_estimator = TaskContext.get 'orientation_estimator'
    Orocos.conf.apply(orientation_estimator, ['default', 'static_log'], :override => true)

    # Create zero velocity measurement
    velocity_writer = orientation_estimator.velocity_samples.writer
    velocity_sample = velocity_writer.new_sample
    velocity_sample.velocity = Types::Base::Vector3d.new(0,0,0)
    cov_velocity = Types::Base::Matrix3d.new
    cov_velocity.data = [10.0, 0, 0,   0, 10.0, 0,   0, 0, 10.0]
    velocity_sample.cov_velocity = cov_velocity

    imu_samples.filter do |sample|
        velocity_sample.time = sample.time
        velocity_writer.write(velocity_sample)
        sample
    end

    imu_samples.connect_to orientation_estimator, :type => :buffer, :size => 1000

    Orocos.transformer.setup(orientation_estimator)

    orientation_estimator.configure
    orientation_estimator.start

    Vizkit.control log
    task_inspector = Vizkit.default_loader.TaskInspector
    Vizkit.display orientation_estimator, :widget => task_inspector
    Vizkit.display orientation_estimator.orientation_samples
    begin
        Vizkit.exec
    rescue Interrupt => e
        orientation_estimator.stop
        orientation_estimator.cleanup
    end
end
