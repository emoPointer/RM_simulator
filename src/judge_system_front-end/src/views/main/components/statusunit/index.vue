<template>
  <div class="container" :style="{'--percentage':!is_left?red_current_percentage.toString()+'%':blue_current_percentage.toString()+'%','--lOrDegree':!is_left?'30deg':'-30deg','--color':!is_left?'rgba(235,56,49, 1)':'rgba(31,150,244,1)','--lOr':!is_left?'left':'right', '--lOrl':!is_left?'-90deg':'90deg'}">
    <div class="red">
      <div class="title">
      {{ title_ }}
      </div>
      <div class="label">
        {{ value_ }}
      </div>
    </div>
  </div>
</template>

<script>
export default {
  props: {
    value: {
      type: Number,
      default: 0
    },
    currentValue: {
      type: Number,
      default: 0
    },
    title: {
      type: String,
      default: 'Null'
    },
    lOr: {
      type: Boolean,
      default: false
    }
  },
  data () {
    return {
      color: null,
      value_: this.value,
      title_: this.title,
      icon_path: '',
      is_invincibility: false,
      is_left: this.lOr,
      current_value: this.currentValue,
      red_current_percentage: 0,
      blue_current_percentage: 0
    }
  },
  watch: {
    value: {
      handler (n, o) {
        this.value_ = n
      }
    },
    currentValue: {
      handler (n, o) {
        this.current_value = n
        this.value_ = n
        this.red_current_percentage = 100 - 100 * (this.current_value / this.value)
        this.blue_current_percentage = 100 - 100 * (this.current_value / this.value)
      }
    }
  }
}
</script>

<style lang="scss" scoped>
  .container {
    --color: rgba(235,56,49, 1);
    --lOrDegree: 30deg;
    --lOr: left;
    --lOrl: 90deg;
    --percentage: 20;
    color: white;
    margin-right: 20px;
    transform: skewX(var(--lOrDegree));
    width: 100px;
    text-align: var(--lOr);
    .label {
      padding-left: 5px;
      background: linear-gradient(var(--lOrl), transparent 0%,transparent var(--percentage),  var(--color) var(--percentage), var(--color) 100%);
      font-size: 24px;
      border-radius: 0 0 3px 3px;
      width: 100px;
    }
    .title {
      padding-left: 5px;
      background: rgba(255, 255, 255, 0.5);
      background: linear-gradient(to var(--lOr), transparent 35%, var(--color));
      font-size: 20px;
      font-weight: 700;
      letter-spacing: 2px;
      border-top-right-radius: 10px;
      border-top-left-radius: 10px;
      width: 100px;
    }
  }
</style>
