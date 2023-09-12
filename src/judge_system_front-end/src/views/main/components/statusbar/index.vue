<template>
  <div class="container">
    <div style="display: flex;">
      <div class="unit-container" v-for="(data, index) in data_list_red" :index="index" :key="index">
      <statusunit :title="data[0]" :value="data_[index*3]" :lOr="false" :current-value="data_[index*3 +1]"/>
    </div>
      <bloodbar :max_value="data_[6]" :lOr="false" :title=data_[9] :current-value="data_[7]"/>
    </div>
    <div>
      <panel :time_minutes="data_[11]" :time_seconds="data_[12]" />
    </div>
    <div style="display: flex;">
      <bloodbar :max_value="data_[6]" :lOr="true"  :title=data_[10] :current-value="data_[8]" />
      <div class="unit-container" v-for="(data, index) in data_list_blue" :index="index" :key="index">
      <statusunit :title="data[0]" :value="data_[index*3]" :lOr="true" :current-value="data_[index*3 +2]"/>
    </div>
    </div>
  </div>
</template>
1 2
4 5
<script>

import statusunit from '../statusunit'
import bloodbar from '../base_blood_bar'
import panel from '../panel'

export default {
  components: {
    statusunit,
    bloodbar,
    panel
  },
  data () {
    return {
      data_: this.data_list,
      data_list_red: [
        ['前哨站', this.data_list[0]],
        ['哨兵', this.data_list[3]]
      ],
      data_list_blue: [
        ['前哨站', this.data_list[0]],
        ['哨兵', this.data_list[3]]
      ]
    }
  },
  props: {
    data_list: {
      type: Array,
      default: () => {
        // OUTPOST GUARD BASE RED_NAME BLUE_NAME TIME_MIN TIME_SEC
        return [
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
        ]
      }
    }
  },
  watch: {
    data_list: {
      handler (n, o) {
        console.log(n, o)
        this.data_ = n
      }
    }
  }
}
</script>

<style lang="scss" scoped>
  .container {
    display: flex;
    justify-content: space-between;
    margin-left: 20px;
    margin-right: 20px;
  }
</style>
