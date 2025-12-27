# Quickstart: Navbar & Auth UI Cleanup

**Feature:** 004-navbar-auth-ui-cleanup
**Audience:** Developers integrating or testing this feature
**Time to Complete:** 5 minutes

---

## Prerequisites

- ✅ Docusaurus 3.9.2 installed
- ✅ Node.js 18+ and npm
- ✅ Working directory: `/my-website/`
- ✅ Phase 3 content deployed (textbook complete)

---

## Quick Integration

### Step 1: Verify Current Navbar

Check the current navbar configuration:

```bash
cd my-website
cat docusaurus.config.ts | grep -A 20 "navbar:"
```

You should see Module 2, 3, 4 links in the navbar items.

---

### Step 2: Test Auth Pages

After implementation, test all routes:

```bash
# Start dev server
npm run start

# Visit these URLs:
# http://localhost:3000/auth
# http://localhost:3000/login
# http://localhost:3000/signup
```

**Expected Behavior:**
- `/auth` → Auth choice page with two buttons
- `/login` → Login form with "Coming soon" notice
- `/signup` → Signup form with "Coming soon" notice
- User icon in navbar → Navigates to `/auth`

---

### Step 3: Verify Build

Ensure production build succeeds:

```bash
npm run build
```

**Success Criteria:**
```
[SUCCESS] Generated static files in "build".
```

**Check generated routes:**
```bash
ls -la build/auth/
ls -la build/login/
ls -la build/signup/
```

---

## Testing Checklist

### Visual Testing

**Navbar (Desktop):**
- [ ] Only "Learning" dropdown visible on left
- [ ] User icon (👤) visible on right (after language/GitHub/theme)
- [ ] No Module 2, 3, 4 links in navbar
- [ ] User icon click navigates to `/auth`

**Navbar (Mobile < 996px):**
- [ ] Hamburger menu works
- [ ] User icon visible in mobile menu
- [ ] All functionality preserved

**Auth Choice Page (`/auth`):**
- [ ] Centered card layout
- [ ] "Welcome!" heading visible
- [ ] Two buttons: "Sign In" and "Sign Up"
- [ ] Buttons navigate correctly
- [ ] Responsive on mobile

**Login Page (`/login`):**
- [ ] Centered form layout
- [ ] Email field (type: email)
- [ ] Password field (type: password, masked)
- [ ] "Login" button (disabled state)
- [ ] "Coming soon" notice displayed
- [ ] Form validates on submit (client-side only)

**Signup Page (`/signup`):**
- [ ] Centered form layout
- [ ] Name, Email, Password fields
- [ ] "Create Account" button (disabled state)
- [ ] "Coming soon" notice displayed
- [ ] Form validates on submit

### Theme Testing

- [ ] Light mode: All pages render correctly
- [ ] Dark mode: All pages render correctly
- [ ] Theme toggle works on all new pages
- [ ] Colors use Infima CSS variables

### Accessibility Testing

- [ ] Tab navigation works through all elements
- [ ] User icon has ARIA label
- [ ] Form fields have associated labels
- [ ] Enter key submits forms
- [ ] Focus indicators visible

### Responsive Testing

Test at these viewport widths:
- [ ] 320px (mobile)
- [ ] 768px (tablet)
- [ ] 996px (Docusaurus breakpoint)
- [ ] 1280px+ (desktop)

---

## Common Issues & Solutions

### Issue 1: Navbar User Icon Not Appearing

**Symptom:** User icon doesn't show after configuration

**Solution:**
1. Clear build cache: `npm run clear`
2. Rebuild: `npm run build`
3. Check `ComponentTypes.tsx` registration
4. Verify `type: 'custom-userIcon'` in config

---

### Issue 2: Auth Pages Return 404

**Symptom:** Navigating to `/auth`, `/login`, `/signup` shows 404

**Solution:**
1. Verify files exist in `src/pages/`:
   ```bash
   ls src/pages/auth.tsx
   ls src/pages/login.tsx
   ls src/pages/signup.tsx
   ```
2. Check file exports: `export default function AuthPage() {}`
3. Rebuild: `npm run build`

---

### Issue 3: Theme Styles Not Applying

**Symptom:** Dark mode doesn't change page colors

**Solution:**
1. Use CSS custom properties: `var(--ifm-background-color)`
2. Check CSS module imports: `import styles from './page.module.css'`
3. Verify selectors: `[data-theme='dark']` for explicit overrides

---

### Issue 4: Mobile Layout Broken

**Symptom:** Pages don't render well on mobile

**Solution:**
1. Use mobile-first CSS (base styles = mobile)
2. Add desktop breakpoint: `@media screen and (min-width: 996px)`
3. Test with browser DevTools responsive mode
4. Use Docusaurus container: `<div className="container">`

---

## File Structure Reference

After implementation, your structure should look like:

```
my-website/
├── docusaurus.config.ts (MODIFIED - navbar config)
├── src/
│   ├── components/
│   │   └── NavbarItems/
│   │       ├── UserIconNavbarItem.tsx (NEW)
│   │       └── UserIconNavbarItem.module.css (NEW)
│   ├── theme/
│   │   └── NavbarItem/
│   │       └── ComponentTypes.tsx (NEW)
│   └── pages/
│       ├── auth.tsx (NEW)
│       ├── auth.module.css (NEW)
│       ├── login.tsx (NEW)
│       ├── login.module.css (NEW)
│       ├── signup.tsx (NEW)
│       └── signup.module.css (NEW)
└── build/ (after npm run build)
    ├── auth/
    ├── login/
    └── signup/
```

---

## Integration with Existing Features

### Sidebar Navigation

**No Changes Required:**
- All modules (0-5) remain accessible via sidebar
- Sidebar automatically generated from docs structure
- No conflicts with navbar changes

### Existing Pages

**No Impact:**
- All doc pages continue to work
- Homepage unchanged
- Module intro pages unaffected

### Build Process

**No Changes Required:**
- Standard Docusaurus build process
- No additional build steps
- No new dependencies

---

## Performance Impact

**Expected Metrics:**

| Metric | Before | After | Change |
|--------|--------|-------|--------|
| Navbar JS | ~5KB | ~6KB | +1KB (UserIcon) |
| Auth Pages | N/A | ~5KB each | +15KB total |
| Build Time | ~20s | ~21s | +1s (3 new pages) |
| First Load | ~200KB | ~201KB | Negligible |

**Optimization:**
- All pages code-split automatically (Docusaurus)
- CSS Modules tree-shaken in production
- No additional network requests
- Icons inlined as SVG

---

## Development Workflow

### Local Development

```bash
# Start dev server with hot reload
npm run start

# Open browser
open http://localhost:3000

# Make changes → Auto-reload
```

### Production Build

```bash
# Clean build
npm run clear
npm run build

# Serve locally
npm run serve

# Test production build
open http://localhost:3000
```

### Deployment

```bash
# Commit changes
git add .
git commit -m "Add navbar cleanup and auth UI pages"

# Push to GitHub
git push origin 004-navbar-auth-ui-cleanup

# Vercel auto-deploys on push to main
# Or merge to main branch for deployment
```

---

## API Reference (Components)

### UserIconNavbarItem Component

```typescript
import UserIconNavbarItem from '@site/src/components/NavbarItems/UserIconNavbarItem';

<UserIconNavbarItem
  mobile={false}        // Desktop mode
  authUrl="/auth"       // Destination URL
/>
```

**Props:**
- `mobile?: boolean` - Responsive mode flag (default: false)
- `authUrl?: string` - Navigation destination (default: '/auth')

---

### Auth Page Layout

```typescript
import Layout from '@theme/Layout';

export default function AuthPage() {
  return (
    <Layout
      title="Authentication"
      description="Login or sign up">
      {/* Your content */}
    </Layout>
  );
}
```

---

## Debugging Tips

### Enable Verbose Logging

```bash
# See all Docusaurus build steps
DEBUG=* npm run build

# Check specific subsystem
DEBUG=docusaurus:* npm run build
```

### Inspect Build Output

```bash
# Check generated HTML
cat build/auth/index.html

# Verify static assets
ls build/assets/
```

### Browser DevTools

1. **Network Tab:** Ensure no 404s for new routes
2. **Console:** Check for React warnings
3. **Responsive Mode:** Test mobile layouts
4. **Lighthouse:** Verify accessibility scores

---

## Next Steps

After successful integration:

1. **Add Backend Authentication** (Future)
   - Integrate Better-Auth or similar
   - Connect forms to API endpoints
   - Add session management

2. **Enhance UX** (Optional)
   - Add loading spinners
   - Implement form validation UI
   - Add password strength indicator

3. **Analytics** (Optional)
   - Track auth page visits
   - Monitor conversion funnel
   - A/B test button labels

---

## Support

**Issue:** Something not working?

**Steps:**
1. Check this quickstart guide
2. Review `/specs/004-navbar-auth-ui-cleanup/plan.md`
3. Verify file structure matches reference
4. Check browser console for errors
5. Run `npm run clear && npm run build`

**Still stuck?**
- Check Docusaurus docs: https://docusaurus.io/docs
- Review implementation in `tasks.md`

---

**Estimated Setup Time:** 5 minutes
**Estimated Testing Time:** 10 minutes
**Total Time to Production:** 15 minutes

✅ **Ready to implement!**
