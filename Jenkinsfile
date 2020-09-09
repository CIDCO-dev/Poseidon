pipeline {

  agent none
  stages {
    stage('run angles test') {
      agent { label 'ubuntu-ros-pi'}
      steps {
        sh 'scripts/test_angles_package'
	junit 'build/test_results/angles/gtest-utest.xml'
      }
    }
  }
}

