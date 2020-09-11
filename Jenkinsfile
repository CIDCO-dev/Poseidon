pipeline {

  agent none
  stages {
    stage('run test on VM') {
      agent { label 'ros-ubuntu-vm'}
      steps {
        sh 'scripts/vm_tests'
	    junit 'build/test_results/angles/gtest-utest.xml'
      }
    }
    /*
    stage('raspberry pi tests') {
      agent { label 'ubuntu-ros-pi'}
      steps {
        sh 'scripts/raspberry_tests'
	    //junit 'build/test_results/angles/gtest-utest.xml'
      }
    }
    */
  }
}

