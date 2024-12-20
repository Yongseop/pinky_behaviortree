import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def load_and_prepare_data():
    amcl_pose = pd.read_csv('amcl_pose_data.csv')
    particle_cloud = pd.read_csv('particle_cloud_data.csv')
    
    merged_data = pd.merge(amcl_pose, particle_cloud, 
                          on=['timestamp', 'initial_pose_set'], 
                          suffixes=('_pose', '_cloud'))
    
    return amcl_pose, particle_cloud, merged_data

def plot_metrics(merged_data):
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(15, 10))
    
    # Initial pose의 첫 설정 시점만 가져오기
    first_initial_pose = merged_data[merged_data['initial_pose_set']]['timestamp'].iloc[0]
    
    # AMCL_POSE 그래프
    ax1.plot(merged_data['timestamp'], merged_data['x_uncertainty'], 
            label='X Uncertainty', color='blue', linewidth=2)
    ax1.plot(merged_data['timestamp'], merged_data['y_uncertainty'], 
            label='Y Uncertainty', color='green', linewidth=2)
    ax1.plot(merged_data['timestamp'], merged_data['theta_uncertainty'], 
            label='Theta Uncertainty', color='red', linewidth=2)
    
    # Initial pose 첫 시점만 표시 (AMCL_POSE)
    ax1.axvline(x=first_initial_pose, color='purple', linestyle='--', alpha=0.7,
               label='Initial Pose')
    
    ax1.set_title('AMCL_POSE Uncertainties', fontsize=14, pad=10)
    ax1.legend(loc='upper right')
    ax1.grid(True)
    ax1.set_ylabel('Uncertainty Value')
    
    # PARTICLE_CLOUD 그래프
    line1 = ax2.plot(merged_data['timestamp'], merged_data['x_variance'], 
                     label='X Variance', color='blue', linewidth=2)
    line2 = ax2.plot(merged_data['timestamp'], merged_data['y_variance'], 
                     label='Y Variance', color='green', linewidth=2)
    
    # 두 번째 y축 생성
    ax2_twin = ax2.twinx()
    line3 = ax2_twin.plot(merged_data['timestamp'], merged_data['max_weight'], 
                         label='Max Weight', color='orange', linewidth=2)
    line4 = ax2_twin.plot(merged_data['timestamp'], merged_data['mean_weight'], 
                         label='Mean Weight', color='red', linewidth=2)
    
    # Initial pose 첫 시점만 표시 (PARTICLE_CLOUD)
    line5 = ax2.axvline(x=first_initial_pose, color='purple', linestyle='--', alpha=0.7,
                       label='Initial Pose')
    
    # 범례 통합
    lines = line1 + line2 + line3 + line4 + [line5]
    labels = [l.get_label() for l in lines]
    ax2.legend(lines, labels, loc='upper right')
    
    ax2.set_title('PARTICLE_CLOUD Metrics', fontsize=14, pad=10)
    ax2.grid(True)
    ax2.set_xlabel('Timestamp')
    ax2.set_ylabel('Variance Value')
    ax2_twin.set_ylabel('Weight Value')
    
    plt.tight_layout()
    plt.show()

def main():
    # 데이터 로드
    amcl_pose, particle_cloud, merged_data = load_and_prepare_data()
    
    # 그래프 표시
    plot_metrics(merged_data)
    
    # Initial pose 설정 시점 분석
    initial_poses = merged_data[merged_data['initial_pose_set']]

if __name__ == "__main__":
    main()
