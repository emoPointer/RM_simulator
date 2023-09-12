<template>
  <div class="container" style="cursor: pointer;" @click="onCountdown">
    <div class="red-socre">
      0
    </div>
    <div class="countdown-container">
      <span class="round">
        Round: -/-
      </span>
      <br>
      <span class="countdown">
        {{ holder_min }}:{{holder_sec}}
      </span>
    </div>
    <div class="blue-socre">
      0
    </div>
  </div>
</template>

<script>
export default {
  data () {
    return {
      time_min: this.time_minutes,
      time_sec: this.time_seconds,
      holder_min: this.time_minutes,
      holder_sec: this.time_seconds,
      total_seconds: 0,
      countdown_timer: null
    }
  },
  props: {
    time_minutes: {
      type: Number,
      default: 0
    },
    time_seconds: {
      type: Number,
      default: 0
    }
  },
  watch: {
    time_minutes: {
      handler (n, o) {
        this.time_min = this.holder_min = n
      }
    },
    time_seconds: {
      handler (n, o) {
        this.time_sec = this.holder_sec = n
      }
    }
  },
  methods: {
    onCountdown () {
      this.total_seconds = this.time_min * 60 + this.time_sec
      this.countdown_timer = setInterval(this.onShowTime, 1000)
    },
    onShowTime () {
      this.total_seconds -= 1
      this.holder_min = Math.floor(this.total_seconds / 60)
      this.holder_sec = this.total_seconds - this.holder_min * 60
      if (this.total_seconds === 0) {
        clearInterval(this.countdown_timer)
      }
    }
  }
}
</script>

<style lang="scss" scoped>
.container {
  .red-socre, .blue-socre {
    width: 50px;
    background:rgba(255, 255, 255, 0.3);
    border: 2px solid rgba(255, 255, 255, 0.5);
    height: 40px;
    display: flex;
    justify-content: center;
    align-items: center;
    color: white;
    font-size: 26px;
    font-weight: 700;
    margin-top: 20px;
    border-radius: 10px;
    box-shadow: 0 0 10px inset rgba(255, 255, 255, 1);
  }
  .red-socre {
    transform: skewX(30deg);
    margin-right: 30px;
  }
  .blue-socre {
    transform: skewX(-30deg);
    margin-left: 30px;
  }
  .countdown-container {
    position: relative;
    width: 100px;
    padding-top: 10px;
    padding-bottom: 10px;
    text-align: center;
    color: white;
    font-weight: 700;
    border-radius: 10px;
    .countdown {
      font-size: 26px;
    }
    .round {
      font-size: 10px;
    }
  }
  .countdown-container::before {
    content: '';
    position: absolute;
    top: 0;
    left: 0;
    right: 0;
    bottom: 0;
    background: rgba(255, 255, 255, 0.3);
    transform: perspective(10px)scale(1.1, 1) rotateX(-5deg);
    z-index: -1;
    border-radius: 10px;
    border: 2px solid rgba(255, 255, 255, 0.5);
  }
}
</style>
