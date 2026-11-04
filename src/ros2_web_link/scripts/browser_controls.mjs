/** Smoke-probe navigation through visible controls, matching the tool shell. */
export async function showTool(page,id){
  if(await page.locator('#connection-panel').isVisible())await page.locator('#connection-close').click();
  await page.locator(`.rail nav a[href="#${id}"]`).click();
  await page.locator(`main > section[id="${id}"]`).waitFor({state:'visible'});
}
export async function showConnection(page){
  if(!await page.locator('#connection-panel').isVisible())await page.locator('#connection-toggle').click();
}
export async function revealControl(page,id){
  const target=page.locator(`#${id}`);
  const location=await target.evaluate(node=>({
    connection:!!node.closest('#connection-panel'),section:node.closest('main > section')?.id,
  }));
  if(location.connection)await showConnection(page);
  else if(location.section)await showTool(page,location.section);
  // Native disclosures can be nested. Open outermost first using summary clicks.
  const details=target.locator('xpath=ancestor::details');
  for(let i=0;i<await details.count();i++){
    const detail=details.nth(i);
    if(!await detail.evaluate(node=>node.open))await detail.locator(':scope > summary').click();
  }
  return target;
}
