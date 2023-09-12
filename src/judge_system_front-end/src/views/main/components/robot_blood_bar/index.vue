<template>
  <div class="container">
    <div class="red-robots">
      <div class="robot-unit" v-for="(data, index) in red_robots_data" :key="index" :index="index" :style="{'--percentage':!is_left?red_current_percentage.toString()+'%':blue_current_percentage.toString()+'%','--lOrDegree':!is_left?'30deg':'-30deg','--color':!is_left?'rgba(235,56,49, 1)':'rgba(31,150,244,1)','--lOr':!is_left?'left':'right','--lOrl':!is_left?'-90deg':'90deg'}">
        <div class="robot-index">
          {{ index+1 }}
        </div>
        <div class="robot-panel">
          <div class="robot-icon">
            <img class="robot-pic" :src="robots_assets_path[data[0]]" />
          </div>
          <div class="robot-blood-bar" />
        </div>
      </div>
    </div>
    <div class="money-panel">
      <money />
    </div>
    <div class="blue-robots">
      <div class="robot-unit" v-for="(data, index) in blue_robots_data" :key="index" :index="index" :style="{'--percentage':!is_left?red_current_percentage.toString()+'%':blue_current_percentage.toString()+'%','--lOrDegree':is_left?'30deg':'-30deg','--color':is_left?'rgba(235,56,49, 1)':'rgba(31,150,244,1)','--lOr':is_left?'left':'right','--lOrl':!is_left?'-90deg':'90deg'}">
        <div class="robot-panel">
          <div class="robot-icon">
            <img class="robot-pic" :src="robots_assets_path[data[0]]" />
          </div>
          <div class="robot-blood-bar" />
        </div>
        <div class="robot-index">
          {{ index+1 }}
        </div>
      </div>
    </div>
  </div>
</template>

<script>

import money from '../money'

export default {
  components: {
    money
  },
  props: {
    currentValue: {
      type: Number,
      default: 0
    }
  },
  watch: {
    current_value: {
      handler (n, o) {
        this.current_value = n
      }
    }
  },
  data () {
    return {
      robots_assets_path: [
        require('@/assets/hero.png'),
        require('@/assets/walker.png'),
        require('@/assets/engineer.png'),
        require('@/assets/drone.png'),
        require('@/assets/guard.png')
      ],
      // type currentValue grade
      red_robots_data: [
        [0, 200, 1],
        [1, 200, 1],
        [2, 200, 1],
        [3, 200, 1],
        [4, 200, 1]
      ],
      blue_robots_data: [
        [0, 200, 1],
        [1, 200, 1],
        [2, 200, 1],
        [3, 200, 1],
        [4, 200, 1]
      ],
      is_left: false,
      current_value: this.currentValue,
      red_current_percentage: 0,
      blue_current_percentage: 0
    }
  }
}
</script>

<style lang="scss" scoped>
.container {
  --color: rgba(235,56,49, 1);
  --lOrDegree: 30deg;
  --percentage: 0%;
  --lOr: left;
  --lOrl: 90deg;
  justify-content: space-between;
  display: flex;
  .robot-index {
    color: white;
    font-size: 24px;
    font-weight: 700;
    display: flex;
    align-items: flex-end;
    margin-right: 5px;
    margin-left: 5px;
  }
  .robot-panel {
    width: 100%;
    .robot-pic{
    height: 50px;
    width: 70px;
  }
  .robot-blood-bar {
    width: 70px;
    background: linear-gradient(var(--lOrl), transparent 0%,transparent var(--percentage),  var(--color) var(--percentage), var(--color) 100%);
    height: 10px;
    border-radius: 5px;
    border: 2px solid rgba(255, 255, 255, 0.2);
    box-shadow: 0 0 10px var(--color) inset,0 0 10px var(--color);
  }
  }
  .red-robots, .blue-robots {
    display: flex;
    .robot-unit {
      display: flex;
      margin-right: 20px;
      transform: skewX(var(--lOrDegree));
    }
  }
}
</style>
