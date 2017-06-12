import { RosControlCenterPage } from './app.po';

describe('ros-control-center App', () => {
  let page: RosControlCenterPage;

  beforeEach(() => {
    page = new RosControlCenterPage();
  });

  it('should display welcome message', () => {
    page.navigateTo();
    expect(page.getParagraphText()).toEqual('Welcome to app!!');
  });
});
