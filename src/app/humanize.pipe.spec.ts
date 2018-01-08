import { HumanizePipe } from './humanize.pipe';

describe('HumanizePipe', () => {
  it('create an instance', () => {
    const pipe = new HumanizePipe();
    expect(pipe).toBeTruthy();
  });
});
