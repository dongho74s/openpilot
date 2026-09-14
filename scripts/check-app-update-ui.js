async (page) => {
  // Synthetic native callbacks; no GitHub/Cloud downloads or installer invocation.
  const check = (value, message) => { if (!value) throw new Error(message); };
  const errors = [];
  page.on('pageerror', error => errors.push(error.message));
  await page.goto('http://127.0.0.1:4198/app/src/main/assets/main.html');
  await page.getByRole('button', {name:'차량', exact:true}).click();
  await page.evaluate(() => {
    window.__updateCalls = [];
    window.Android = {
      checkAppUpdate(){ window.__updateCalls.push('check'); onHylinkUpdateState({state:'checking', message:'새 버전을 확인하고 있어요.'}); },
      downloadAppUpdate(){ window.__updateCalls.push('download'); },
      installAppUpdate(){ window.__updateCalls.push('install'); },
      cancelAppUpdate(){ window.__updateCalls.push('cancel'); },
    };
  });
  await page.getByRole('button', {name:'업데이트 확인', exact:true}).click();
  check(await page.getByRole('button', {name:'확인 중…', exact:true}).isDisabled(), 'Checking is non-duplicable');
  let layouts = 0;
  for (const width of [320,390,768]) for (const scale of [1,2]) for (const dark of [false,true]) {
    await page.setViewportSize({width,height:844});
    await page.evaluate(({scale,dark}) => {
      onHylinkFontScale(scale);
      document.body.classList.toggle('dark',dark);
      onHylinkUpdateState({state:'available',currentVersion:'1.6.5-app-updates-debug',latestVersion:'1.6.6-app-updates-debug',
        hasUpdate:true,message:'새 버전을 사용할 수 있어요.',notes:'앱 업데이트 동작을 개선했어요.\n기존 연결 키와 데이터는 유지돼요.'});
    }, {scale,dark});
    await page.locator('#app-update-card').scrollIntoViewIfNeeded();
    check(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth), 'No horizontal overflow');
    const bounds = await page.locator('#app-update-button').boundingBox();
    check(bounds.height >= 48 && bounds.width >= 48, 'Comfortable touch target');
    check(await page.evaluate(() => {
      const linear = c => { c/=255; return c<=.04045 ? c/12.92 : ((c+.055)/1.055)**2.4; };
      const luminance = color => {const x=color.match(/[\d.]+/g).slice(0,3).map(Number).map(linear);return .2126*x[0]+.7152*x[1]+.0722*x[2];};
      const card=getComputedStyle(document.getElementById('app-update-card'));
      const label=getComputedStyle(document.getElementById('app-update-status'));
      const a=luminance(card.backgroundColor),b=luminance(label.color);
      return (Math.max(a,b)+.05)/(Math.min(a,b)+.05)>=4.5;
    }), 'Status text meets 4.5:1 contrast');
    layouts++;
  }
  await page.getByRole('button', {name:'업데이트 다운로드',exact:true}).click();
  check(await page.evaluate(() => window.__updateCalls.at(-1)==='download'), 'Native download action');
  await page.evaluate(() => onHylinkUpdateState({state:'downloading',progress:42,message:'업데이트를 다운로드하고 있어요.'}));
  check(await page.getByRole('progressbar').getAttribute('value')==='42', 'Real determinate progress');
  await page.getByRole('button', {name:'취소',exact:true}).click();
  check(await page.evaluate(() => window.__updateCalls.at(-1)==='cancel'), 'Native cancellation');
  await page.evaluate(() => onHylinkUpdateState({state:'ready',message:'검증 완료 · 설치 화면에서 한 번 더 확인해 주세요.'}));
  await page.getByRole('button', {name:'업데이트 설치',exact:true}).click();
  check(await page.evaluate(() => window.__updateCalls.at(-1)==='install'), 'Explicit install action');
  await page.evaluate(() => onHylinkUpdateState({state:'error',message:'네트워크 연결을 확인하고 다시 시도해 주세요.'}));
  check(await page.getByRole('button', {name:'다시 확인',exact:true}).isEnabled(), 'Recoverable network error');
  await page.setViewportSize({width:390,height:844});
  await page.evaluate(() => {onHylinkFontScale(1);document.body.classList.remove('dark');onHylinkUpdateState({state:'current',currentVersion:'1.6.5-app-updates-debug',message:'최신 버전이에요.'});});
  await page.locator('#app-update-card').scrollIntoViewIfNeeded();
  await page.screenshot({path:'output/playwright/app-update-vehicle.png'});
  check(errors.length===0, 'No JS exceptions: '+errors.join(', '));
  return 'PASS native action wiring, errors/progress/cancel, '+layouts+' layouts, 200% text and light/dark contrast';
}
