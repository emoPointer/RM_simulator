<template>
  <div class="container">
    <video class="bg-video" muted autoplay="autoplay" loop="loop" src="../../assets/video.mp4">
    </video>
    <div class="top-panel" style="cursor: pointer;" @click="showDialog()" >
      <statusbar class="statusbar" :data_list="statusbar_data" />
      <robot-blood-bar class="robot-blood-bar"/>
    </div>
    <minimap class="mini-map" />
    <log class="log"/>
    <el-dialog
      class="dialog"
      title="全局数值设置"
      :visible.sync="dialogVisible"
      width="50%"
      :close-on-click-modal="false"
      >
      <div class="main-panel">
              <div class="left-panel">
        <div class="input-container">
        <div class="title">比赛持续时间</div>
        <div class="input">
          <el-input v-model="minutes" min="0" type="number"/> <span>分</span>
          <el-input v-model="seconds" min="0" max="59" type="number"/> <span>秒</span>
        </div>
      </div>
      <div class="input-container">
        <div class="title">基地最大血量</div>
        <div class="input">
          <el-input v-model="base_blood" min="0" type="number"/>
        </div>
      </div>
      <div class="input-container">
        <div class="title">前哨站最大血量</div>
        <div class="input">
          <el-input v-model="outpost_blood" min="0" type="number"/>
        </div>
      </div>
      <div class="input-container">
        <div class="title">哨兵最大血量</div>
        <div class="input">
          <el-input v-model="guard_blood" min="0" type="number"/>
        </div>
      </div>
      <div class="input-container">
        <div class="title">步兵最大血量</div>
        <div class="input">
          <el-input v-model="walker_blood" min="0" type="number"/>
        </div>
      </div>
      <div class="input-container">
        <div class="title">英雄最大血量</div>
        <div class="input">
          <el-input v-model="hero_blood" min="0" type="number"/>
        </div>
      </div>
      <div class="input-container">
        <div class="title">无人机最大血量</div>
        <div class="input">
          <el-input v-model="drone_blood" min="0" type="number"/>
        </div>
      </div>
      <div class="input-container">
        <div class="title">工程最大血量</div>
        <div class="input">
          <el-input v-model="eng_blood" min="0" type="number"/>
        </div>
      </div>
      <div class="input-container">
        <div class="title">红方标题</div>
        <div class="input">
          <el-input v-model="red_name"/>
        </div>
      </div>
      <div class="input-container">
        <div class="title">蓝方标题</div>
        <div class="input">
          <el-input v-model="blue_name"/>
        </div>
      </div>
      </div>
      <div class="right-panel">
        <div class="input-container">
        <div class="title">红方基地血量</div>
        <div class="input">
          <el-input v-model="red_blood_array[0]" min="0" :max="base_blood" type="number"/>
        </div>
      </div>
      <div class="right-panel">
        <div class="input-container">
        <div class="title">红方前哨站血量</div>
        <div class="input">
          <el-input v-model="red_blood_array[1]" min="0" :max="base_blood" type="number"/>
        </div>
      </div>
      <div class="input-container">
        <div class="title">红方哨兵血量</div>
        <div class="input">
          <el-input v-model="red_blood_array[2]" min="0" :max="guard_blood" type="number"/>
        </div>
      </div>
      <div class="input-container">
        <div class="title">红方英雄血量</div>
        <div class="input">
          <el-input v-model="red_blood_array[3]" min="0" :max="hero_blood" type="number"/>
        </div>
      </div>
      <div class="input-container">
        <div class="title">红方步兵血量</div>
        <div class="input">
          <el-input v-model="red_blood_array[4]" min="0" :max="guard_blood" type="number"/>
        </div>
      </div>
      <div class="input-container">
        <div class="title">红方工程血量</div>
        <div class="input">
          <el-input v-model="red_blood_array[5]" min="0" :max="eng_blood" type="number"/>
        </div>
      </div>
      <div class="input-container">
        <div class="title">红方无人机血量</div>
        <div class="input">
          <el-input v-model="red_blood_array[6]" min="0" :max="drone_blood" type="number"/>
        </div>
      </div>
      <div class="input-container">
        <div class="title">蓝方基地血量</div>
        <div class="input">
          <el-input v-model="blue_blood_array[0]" min="0" :max="base_blood" type="number"/>
        </div>
      </div>
      <div class="input-container">
        <div class="title">蓝方前哨站血量</div>
        <div class="input">
          <el-input v-model="blue_blood_array[1]" min="0" :max="eng_blood" type="number"/>
        </div>
      </div>
      <div class="input-container">
        <div class="title">蓝方哨兵血量</div>
        <div class="input">
          <el-input v-model="blue_blood_array[2]" min="0" :max="guard_blood" type="number"/>
        </div>
      </div>
      <div class="input-container">
        <div class="title">蓝方英雄血量</div>
        <div class="input">
          <el-input v-model="blue_blood_array[3]" min="0" :max="hero_blood" type="number"/>
        </div>
      </div>
      <div class="input-container">
        <div class="title">蓝方步兵血量</div>
        <div class="input">
          <el-input v-model="red_blood_array[4]" min="0" :max="eng_blood" type="number"/>
        </div>
      </div>
      <div class="input-container">
        <div class="title">蓝方工程血量</div>
        <div class="input">
          <el-input v-model="blue_blood_array[5]" min="0" :max="eng_blood" type="number"/>
        </div>
      </div>
      <div class="input-container">
        <div class="title">蓝方无人机血量</div>
        <div class="input">
          <el-input v-model="blue_blood_array[6]" min="0" :max="drone_blood" type="number"/>
        </div>
      </div>
      </div>
      </div>
      </div>
      <div class="button-panel">
        <el-button @click="onUpdate">确定更改</el-button>
      </div>
    </el-dialog>
  </div>
</template>
<script>

import statusbar from './components/statusbar'
import robotBloodBar from './components/robot_blood_bar'
import minimap from './components/map'
import log from './components/log'
import axios from 'axios'

export default {
  components: {
    statusbar,
    robotBloodBar,
    minimap,
    log
  },
  data () {
    return {
      dialogVisible: false,
      minutes: 0,
      seconds: 0,
      base_blood: 3000,
      guard_blood: 1500,
      hero_blood: 1000,
      eng_blood: 1000,
      drone_blood: 1000,
      outpost_blood: 2000,
      walker_blood: 1000,
      red_name: '红方',
      blue_name: '蓝方',
      // BASE OUTPOST GUARD HERO WALKER ENG DRONE
      red_blood_array: [3000, 2000, 1500, 1000, 1000, 1000, 1000],
      blue_blood_array: [3000, 2000, 1500, 1000, 1000, 1000, 1000],
      statusbar_data: [
        2000,
        2000,
        2000,
        1500,
        1500,
        1500,
        3000,
        3000,
        3000,
        '红方',
        '蓝方',
        0,
        0
      ],
      keys: [
        'remaining_time',
        'blue_base_blood',
        'red_base_blood',
        'r1_blood',
        'b1_blood',
        'r2_blood',
        'b2_blood',
        'r3_blood',
        'b3_blood',
        'r4_blood',
        'b4_blood',
        'r5_blood',
        'b5_blood',
        'r6_blood',
        'b6_blood',
        'r7_blood',
        'b7_blood',
        'r_outpost',
        'b_outpost',
        'energy',
        'r_darts',
        'b_darts',
        'bullet_dose'
      ],
      global_data: [
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0
      ]
    }
  },
  methods: {
    showDialog () {
      this.dialogVisible = true
    },
    onUpdate () {
      this.dialogVisible = false
      this.global_data = [
        parseInt(this.minutes) * 60 + parseInt(this.seconds),
        this.blue_blood_array[0],
        this.red_blood_array[0],
        this.red_blood_array[3],
        this.blue_blood_array[3],
        this.red_blood_array[5],
        this.blue_blood_array[5],
        this.red_blood_array[4],
        this.blue_blood_array[4],
        this.red_blood_array[4],
        this.blue_blood_array[4],
        this.red_blood_array[4],
        this.blue_blood_array[4],
        this.red_blood_array[6],
        this.blue_blood_array[6],
        this.red_blood_array[3],
        this.blue_blood_array[3],
        this.red_blood_array[1],
        this.blue_blood_array[1],
        0,
        0,
        0,
        0
      ]
      this.statusbar_data = [
        this.outpost_blood,
        this.red_blood_array[1],
        this.blue_blood_array[1],
        this.guard_blood,
        this.red_blood_array[2],
        this.blue_blood_array[2],
        this.base_blood,
        this.red_blood_array[0],
        this.blue_blood_array[0],
        this.red_name,
        this.blue_name,
        parseInt(this.minutes),
        parseInt(this.seconds)
      ]
      var jsonData = {}
      for (let i = 0; i < this.keys.length; i++) {
        jsonData[this.keys[i]] = this.global_data[i]
      }
      axios.post('http://192.168.25.128:1111', JSON.stringify(jsonData)).then(response => {
        console.log(response.data)
      })
        .catch(error => {
          console.error(error)
        })
    }
  }
}
</script>

<style lang="scss" scoped>
.container {
  position: relative;
  height: 100%;
  justify-content: space-between;
  .dialog{
    font-family: Harmony;
    .main-panel {
      display: flex;
      justify-content: space-between;
      height: 100%;
    }
    .input-container {
      display: flex;
      align-items: center;
      margin-bottom: 10px;
      .title {
        width: calc(15px * 7);
        margin-right: 10px;
        font-size: 15px;
        display: flex;
        justify-content: right;
      }
      .input {
        display: flex;
        align-items: center;
        span {
          margin-left: 5px;
          margin-right: 5px;
        }
        .el-input {
          width: 100px;
        }
      }
    }
  }
  .statusbar, .robot-blood-bar {
    margin: 5px;
  }
  .mini-map {
    position: fixed;
    top: 70%;
    left: 75%;
  }
  .log {
    position: fixed;
    top: 70%;
    left: 5%;
  }
}
 .container video{
  position: absolute;
  width: 100%;
  z-index: -1;
  filter: blur(20px);
 }
</style>
