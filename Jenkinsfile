pipeline {

  agent none
  stages {
    stage('run angles test') {
      agent { label 'ros-ubuntu-vm'}
      steps {
        sh 'scripts/test_angles_package'
	    junit 'build/test_results/angles/gtest-utest.xml'
      }
    }
    stage('raspberry pi tests') {
      agent { label 'ubuntu-ros-pi'}
      steps {
        sh 'scripts/test_angles_package'
	    junit 'build/test_results/angles/gtest-utest.xml'
      }
    }
  }
}

