<template>
  <div class="container">
    <div class="red-money">
      500
    </div>
    <div class="money-icon">
      <svg xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" width="30" height="30" viewBox="0 0 60 60"><defs><path id="a" d="M0 0h59.9999844v60H0z"/></defs><g fill="none" fill-rule="evenodd"><path d="M44.7290026 38h-4.2709975v-2.8321508h4.2709975c.7838741 0 1.4261472-.6325728 1.4261472-1.4197412v-8.4962948c0-.7803098-.6354623-1.4196624-1.4261472-1.4196624H25.9959311V21h18.7330715C47.0807042 21 49 22.9040252 49 25.2514979v8.4966101c0 2.3478668-1.9192958 4.2515767-4.2709974 4.2515767V38zm-9.9612522 0H16.2713142C13.9192958 38 12 36.0959748 12 33.748108v-8.4962948c0-2.3410082 1.9128018-4.2514979 4.2713142-4.2514979h4.2709974v2.8321508h-4.2709974c-.7838741 0-1.4261472.6322575-1.4261472 1.4193471v8.4966889c0 .7803098.6354623 1.4193471 1.4261472 1.4193471h18.4964362V38z" fill="#C6B287"/><path d="M34.7481868 48h-8.4963736c-2.3410082 0-4.2514979-1.9128018-4.2514979-4.2710766v-3.554756h2.8321508v3.554756c0 .7839533.6322575 1.4258305 1.4193471 1.4258305h8.4966889c.7803098 0 1.4193471-.6353832 1.4193471-1.4258305V25.2322496H39v18.4966738C39 46.0807042 37.0895103 48 34.7485021 48h-.0003153zm-9.916036-14.468964H22V15.2709974C22 12.9192958 23.9040252 11 26.2518132 11h8.4963736c2.3410082 0 4.2514979 1.912485 4.2514979 4.2709974v4.1494328h-2.8321508v-4.1494328c0-.7838741-.6322575-1.4261472-1.4193471-1.4261472h-8.4966889c-.7803098 0-1.4193471.6354623-1.4193471 1.4261472V33.531036z" fill="#C6B287"/><mask id="b" fill="#fff"><use xlink:href="#a"/></mask><path d="M29.9999844 3.0932812C15.166625 3.0932812 3.0932656 15.159922 3.0932656 30S15.166625 56.9067187 29.9999844 56.9067187 56.9134219 44.8333594 56.9134219 30c0-14.8333594-12.0733594-26.9067187-26.9134375-26.9067187m0 56.9067187c-16.5399219 0-30-13.4600781-30-30s13.4600781-30 30-30 30 13.4600781 30 30-13.4533594 30-30 30" fill="#C6B287" mask="url(#b)"/></g></svg>
    </div>
    <div class="blue-money">
      500
    </div>
  </div>
</template>

<script>
export default {

}
</script>

<style lang="scss" scoped>
  .container {
    height: 40px;
    display: flex;
    justify-content: center;
    align-items: center;
    .money-icon {
      background: rgba(0, 0, 0, 0.6);
      display: flex;
      justify-content: center;
      align-items: center;
      height: calc(100% + 5px);
      box-shadow: 0 0 5px 5px rgba(0, 0, 0, 0.6);
      z-index: 5;
    }
    .red-money, .blue-money {
      position: relative;
      color: white;
      font-size: 20px;
      font-weight: 700;
      background: rgba(111, 111, 111, 0.5);
      line-height: 40px;
      width: 50px;
      text-align: center;
    }
    .red-money::before {
      content: '';
      position: absolute;
      top: 0;
      left: -20px;
      border-top: 20px solid transparent;
      border-right: 20px solid rgba(235,56,49, 1);
      border-bottom: 20px solid transparent;
    }
    .blue-money::after {
      content: '';
      position: absolute;
      top: 0;
      right: -20px;
      border-top: 20px solid transparent;
      border-left: 20px solid rgba(31,150,244,1);
      border-bottom: 20px solid transparent;
    }
  }
</style>
