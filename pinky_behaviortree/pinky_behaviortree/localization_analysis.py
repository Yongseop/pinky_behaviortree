import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def load_and_prepare_data():
    amcl_pose = pd.read_csv('amcl_pose_data.csv')
    return amcl_pose

def plot_metrics(amcl_data):
    fig, ax = plt.subplots(figsize=(15, 6))
    
    # Initial pose의 첫 설정 시점 가져오기
    first_initial_pose = amcl_data[amcl_data['initial_pose_set']]['timestamp'].iloc[0]
    
    # AMCL_POSE 그래프
    ax.plot(amcl_data['timestamp'], amcl_data['x_uncertainty'], 
            label='X Uncertainty', color='blue', linewidth=2)
    ax.plot(amcl_data['timestamp'], amcl_data['y_uncertainty'], 
            label='Y Uncertainty', color='green', linewidth=2)
    ax.plot(amcl_data['timestamp'], amcl_data['theta_uncertainty'], 
            label='Theta Uncertainty', color='red', linewidth=2)
    
    # Initial pose 첫 시점 표시
    ax.axvline(x=first_initial_pose, color='purple', linestyle='--', alpha=0.7,
               label='Initial Pose')
    
    ax.set_title('AMCL_POSE Uncertainties', fontsize=14, pad=10)
    ax.legend(loc='upper right')
    ax.grid(True)
    ax.set_xlabel('Timestamp')
    ax.set_ylabel('Uncertainty Value')
    
    plt.tight_layout()
    plt.show()

def main():
    # 데이터 로드
    amcl_data = load_and_prepare_data()
    
    # 그래프 표시
    plot_metrics(amcl_data)
    
    # Initial pose 설정 시점 분석
    initial_poses = amcl_data[amcl_data['initial_pose_set']]

if __name__ == "__main__":
    main()