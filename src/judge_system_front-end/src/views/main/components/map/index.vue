<template>
  <div class="container">
    <div class="map-container">
      Mini Map
    </div>
  </div>
</template>

<script>
export default {
  data () {
    return {
      draw_points: []
    }
  }
}
</script>

<style lang="scss" scoped>
.container {
  width: 300px;
  height: 150px;
  border-radius: 5px;
  background: rgba(255, 255, 255, 0.6);
  display: flex;
  justify-content: center;
  align-items: center;
}
</style>
