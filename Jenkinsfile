pipeline {

  agent none
  stages {
    stage('run angles test') {
      agent { label 'ros-ubuntu-vm'}
      steps {
        sh 'scripts/test_angles_package'
      }
    }
  }
}

