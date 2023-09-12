<template>
  <div class="container" :style="{'--lOrDegree':!is_left?'30deg':'-30deg','--color':!is_left?'rgba(235,56,49, 1)':'rgba(31,150,244,1)','--lOr':!is_left?'left':'right'}">
    <div v-if="is_left">
      <div class="blood_icon">
        <svg xmlns="http://www.w3.org/2000/svg" width="50" height="50" viewBox="0 0 88 88"><path d="M70.8341742 31.579374l-6.1264839 6.1580869 2.9062258 2.9212174-2.9062258 2.9193043L67.6139161 46.4992l-2.9062258 2.9212174 6.1264839 6.1580869 1.9393871-1.9493913-4.189-4.2086956L71.4907871 46.4992l-2.9062258-2.9212174 2.9062258-2.9193043-2.9062258-2.9212174 4.189-4.2086957zm-52.7197355 0l-1.939387 1.9493912 4.1909031 4.2086957-2.908129 2.9212174 2.908129 2.9193043-2.908129 2.9212174 2.908129 2.9212174-4.1909032 4.2086956 1.9393871 1.9493913 6.1283871-6.1580869L21.3366 46.4992l2.9062258-2.9212174L21.3366 40.6586783l2.9062258-2.9212174zm37.6667419 19.6180695l-11.3051612 8.6737391-11.3070646-8.6737391-1.652-18.3881739 8.9413549-5.6798261h8.0335161l8.9394516 5.6798261-1.6500968 18.3881739zM39.4001161 23.454487l-11.7257742 7.4474782 1.9964839 22.2372174 14.8051936 11.3539131 14.8032903-11.3539131 1.9964838-22.2372174-11.7257741-7.4474782H39.4001161z" fill="#FFF" fill-rule="evenodd"/></svg>
      </div>
    </div>
    <div class="panel">
      <div class="title">
        {{ bar_title }}
      </div>
      <div class="blood_bar" />
    </div>
    <div v-if="!is_left">
      <div class="blood_icon">
        <svg xmlns="http://www.w3.org/2000/svg" width="50" height="50" viewBox="0 0 88 88"><path d="M70.8341742 31.579374l-6.1264839 6.1580869 2.9062258 2.9212174-2.9062258 2.9193043L67.6139161 46.4992l-2.9062258 2.9212174 6.1264839 6.1580869 1.9393871-1.9493913-4.189-4.2086956L71.4907871 46.4992l-2.9062258-2.9212174 2.9062258-2.9193043-2.9062258-2.9212174 4.189-4.2086957zm-52.7197355 0l-1.939387 1.9493912 4.1909031 4.2086957-2.908129 2.9212174 2.908129 2.9193043-2.908129 2.9212174 2.908129 2.9212174-4.1909032 4.2086956 1.9393871 1.9493913 6.1283871-6.1580869L21.3366 46.4992l2.9062258-2.9212174L21.3366 40.6586783l2.9062258-2.9212174zm37.6667419 19.6180695l-11.3051612 8.6737391-11.3070646-8.6737391-1.652-18.3881739 8.9413549-5.6798261h8.0335161l8.9394516 5.6798261-1.6500968 18.3881739zM39.4001161 23.454487l-11.7257742 7.4474782 1.9964839 22.2372174 14.8051936 11.3539131 14.8032903-11.3539131 1.9964838-22.2372174-11.7257741-7.4474782H39.4001161z" fill="#FFF" fill-rule="evenodd"/></svg>
      </div>
    </div>
  </div>
</template>

<script>
export default {
  props: {
    max_value: {
      type: Number,
      default: 0
    },
    currentValue: {
      type: Number,
      default: 0
    },
    lOr: {
      type: Boolean,
      default: false
    },
    title: {
      type: String,
      default: ''
    }
  },
  data () {
    return {
      is_left: this.lOr,
      blood_value: this.max_value,
      bar_title: this.title,
      current_value: this.currentValue
    }
  },
  watch: {
    max_value: {
      handler (n, o) {
        this.blood_value = n
      }
    },
    title: {
      handler (n, o) {
        this.bar_title = n
      }
    },
    currentValue: {
      handler (n, o) {
        this.current_value = n
      }
    }
  }
}
</script>

<style lang="scss" scoped>
.container{
  display: flex;
  --color: rgba(235,56,49, 1);
  --lOrDegree: 30deg;
  --lOr: left;
  .title {
    font-size: 20px;
    font-weight: 700;
    text-align: var(--lOr);
    color: white;
  }
  .blood_bar {
    width: 200px;
    height: 30px;
    background: var(--color);
    transform: skewX(var(--lOrDegree));
    border-radius: 3px;
    border: 2px solid rgba(255, 255, 255, 0.2);
    box-shadow: 0 0 10px var(--color) inset,0 0 10px var(--color);
  }
  .blood_icon {
    border-radius: 50%;
    height: 50px;
    width: 50px;
    background: var(--color);
  }
}
</style>
